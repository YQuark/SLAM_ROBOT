#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <math.h>
#include <EEPROM.h>
#include <string.h>

// -------------------- Wi-Fi --------------------
static const char *AP_SSID = "ESP01S_CTRL";
static const char *AP_PASS = "12345678";
ESP8266WebServer server(80);
static char g_staTargetSsid[33] = {0};
static bool g_sta_connecting = false;
static uint32_t g_sta_connect_start_ms = 0;

static const uint32_t WIFI_CFG_MAGIC = 0xA55A5AA5u;
typedef struct {
  uint32_t magic;
  char ssid[33];
  char pass[65];
  uint16_t crc;
} wifi_cfg_t;

// -------------------- UART Protocol --------------------
static const uint8_t PROTO_SOF = 0xA5;
static const uint8_t PROTO_EOF = 0x5A;
static const uint8_t PROTO_VER = 0x01;

static const uint8_t FLAG_ACK_REQ = 0x01;
static const uint8_t FLAG_IS_ACK = 0x02;
static const uint8_t FLAG_IS_NACK = 0x04;

static const uint8_t MSG_CMD_PING = 0x01;
static const uint8_t MSG_CMD_GET_STATUS = 0x02;
static const uint8_t MSG_CMD_SET_DRIVE = 0x10;
static const uint8_t MSG_CMD_SET_MODE = 0x11;
static const uint8_t MSG_CMD_SET_IMU = 0x12;
static const uint8_t MSG_CMD_GET_DIAG = 0x30;
static const uint8_t MSG_CMD_CLEAR_DIAG = 0x31;
static const uint8_t MSG_CMD_GET_FAULT_LOG = 0x32;
static const uint8_t MSG_CMD_CLEAR_FAULT_LOG = 0x33;
static const uint8_t MSG_ACK = 0x80;
static const uint8_t MSG_NACK = 0x81;

static const uint16_t PROTO_ERR_OK = 0x0000;

static const uint16_t FRAME_MAX = 260;
static const uint16_t COBS_MAX = 300;

static uint8_t g_seq = 1;

struct AckState {
  bool ready;
  bool nack;
  uint8_t seq;
  uint16_t err;
  uint8_t detail;
  uint8_t extra[220];
  uint16_t extraLen;
};

static AckState g_ack = {false, false, 0, 0, 0, {0}, 0};

struct StatusData {
  uint32_t uptimeMs;
  uint8_t mode;
  uint8_t src;
  uint16_t vbMv;
  uint16_t pctX10;
  int16_t vQ15;
  int16_t wQ15;
  int16_t gzX10;
  uint8_t imuEnabled;
  uint8_t imuValid;
  int16_t encVel[4];  // 缂傚倸鍊搁崐褰掓偋濠婂牊鍋夋繝濠傜墕闂傤垶鏌曟繛鐐澒闁稿鎹囬幃銏犵暋闁箑鏁?(counts/s)
  uint32_t lastRxMs;
};

static StatusData g_status = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static char g_statusJson[320] = "{}";

static uint8_t g_rxAccum[COBS_MAX];
static uint16_t g_rxAccumLen = 0;

static float g_lastCmdV = 0.0f;
static float g_lastCmdW = 0.0f;
static uint32_t g_lastCmdTxMs = 0;
static float g_pendingCmdV = 0.0f;
static float g_pendingCmdW = 0.0f;
static bool g_cmdDirty = false;
static float g_txCmdV = 0.0f;
static float g_txCmdW = 0.0f;
static char g_resetReason[64] = {0};
static uint32_t g_lastHttpCmdMs = 0;
static float g_lastHttpCmdV = 0.0f;
static float g_lastHttpCmdW = 0.0f;
static uint32_t g_lastHttpRxMs = 0;
static uint32_t g_uartTxTimeouts = 0;
static uint32_t g_loopMaxGapMs = 0;
static uint32_t g_loopLastMs = 0;

static const uint32_t WDT_TIMEOUT_MS = 8000;
static const uint32_t DRIVE_SEND_PERIOD_MS = 40;  // Reduce command TX pressure
static const uint32_t CMD_IDLE_STOP_MS = 250;     // Stop quickly after HTTP command loss
static const uint32_t STATUS_POLL_MS = 2000;  // Reduce status polling frequency
static const uint32_t STA_CONNECT_TIMEOUT_MS = 15000;
static const uint32_t CMD_DUP_FILTER_MS = 100;
static const uint16_t SERIAL_RX_BUDGET_BYTES = 256;
static const uint32_t UART_WRITE_TIMEOUT_MS = 8;

static uint8_t nextSeq() {
  uint8_t seq = g_seq++;
  if (g_seq == 0) g_seq = 1;
  return seq;
}

static void clearAckState() {
  g_ack.ready = false;
  g_ack.nack = false;
  g_ack.seq = 0;
  g_ack.err = 0;
  g_ack.detail = 0;
  g_ack.extraLen = 0;
}

// -------------------- Utils --------------------
static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void putU16LE(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void putU32LE(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static uint16_t getU16LE(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t getU32LE(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t crc16CcittFalse(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
      else crc <<= 1;
    }
  }
  return crc;
}

static uint16_t crc16Raw(const uint8_t *data, uint16_t len) {
  return crc16CcittFalse(data, len);
}

static bool loadWifiCfg(wifi_cfg_t *cfg) {
  if (!cfg) return false;
  EEPROM.get(0, *cfg);
  if (cfg->magic != WIFI_CFG_MAGIC) return false;
  uint16_t calc = crc16Raw((const uint8_t*)cfg, (uint16_t)(sizeof(wifi_cfg_t) - sizeof(uint16_t)));
  if (calc != cfg->crc) return false;
  if (cfg->ssid[0] == '\0') return false;
  cfg->ssid[32] = '\0';
  cfg->pass[64] = '\0';
  return true;
}

static bool saveWifiCfg(const char *ssid, const char *pass) {
  if (!ssid || ssid[0] == '\0') return false;
  wifi_cfg_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = WIFI_CFG_MAGIC;
  strncpy(cfg.ssid, ssid, sizeof(cfg.ssid) - 1);
  if (pass) strncpy(cfg.pass, pass, sizeof(cfg.pass) - 1);
  cfg.crc = crc16Raw((const uint8_t*)&cfg, (uint16_t)(sizeof(wifi_cfg_t) - sizeof(uint16_t)));
  EEPROM.put(0, cfg);
  return EEPROM.commit();
}

static uint16_t cobsEncode(const uint8_t *input, uint16_t len, uint8_t *output, uint16_t outMax) {
  uint16_t readIndex = 0;
  uint16_t writeIndex = 1;
  uint16_t codeIndex = 0;
  uint8_t code = 1;

  if (outMax == 0) return 0;

  while (readIndex < len) {
    if (writeIndex >= outMax) return 0;

    if (input[readIndex] == 0) {
      output[codeIndex] = code;
      code = 1;
      codeIndex = writeIndex++;
      readIndex++;
    } else {
      output[writeIndex++] = input[readIndex++];
      code++;
      if (code == 0xFF) {
        output[codeIndex] = code;
        code = 1;
        codeIndex = writeIndex++;
        if (writeIndex > outMax) return 0;
      }
    }
  }

  if (codeIndex >= outMax) return 0;
  output[codeIndex] = code;
  return writeIndex;
}

static uint16_t cobsDecode(const uint8_t *input, uint16_t len, uint8_t *output, uint16_t outMax) {
  uint16_t readIndex = 0;
  uint16_t writeIndex = 0;

  while (readIndex < len) {
    uint8_t code = input[readIndex++];
    if (code == 0) return 0;

    for (uint8_t i = 1; i < code; ++i) {
      if (readIndex >= len || writeIndex >= outMax) return 0;
      output[writeIndex++] = input[readIndex++];
    }

    if (code != 0xFF && readIndex < len) {
      if (writeIndex >= outMax) return 0;
      output[writeIndex++] = 0;
    }
  }

  return writeIndex;
}

static void buildStatusJson() {
  float vBatt = g_status.vbMv / 1000.0f;
  float pct = g_status.pctX10 / 10.0f;
  float vCmd = g_status.vQ15 / 32767.0f;
  float wCmd = g_status.wQ15 / 32767.0f;
  float gz = g_status.gzX10 / 10.0f;

  snprintf(g_statusJson, sizeof(g_statusJson),
           "{\"uptime_ms\":%lu,\"mode\":%u,\"src\":%u,"
           "\"vbatt\":%.3f,\"percent\":%.1f,\"v\":%.3f,\"w\":%.3f,"
           "\"imu\":{\"gz\":%.1f,\"enabled\":%u,\"valid\":%u},"
           "\"enc\":[%d,%d,%d,%d]}",
           (unsigned long)g_status.uptimeMs,
           (unsigned)g_status.mode,
           (unsigned)g_status.src,
           vBatt,
           pct,
           vCmd,
           wCmd,
           gz,
           (unsigned)g_status.imuEnabled,
           (unsigned)g_status.imuValid,
           (int)g_status.encVel[0],
           (int)g_status.encVel[1],
           (int)g_status.encVel[2],
           (int)g_status.encVel[3]);
}

static bool parseStatusExtra(const uint8_t *data, uint16_t len) {
  if (len < 18) return false;

  g_status.uptimeMs = getU32LE(&data[0]);
  g_status.mode = data[4];
  g_status.src = data[5];
  g_status.vbMv = getU16LE(&data[6]);
  g_status.pctX10 = getU16LE(&data[8]);
  g_status.vQ15 = (int16_t)getU16LE(&data[10]);
  g_status.wQ15 = (int16_t)getU16LE(&data[12]);
  g_status.gzX10 = (int16_t)getU16LE(&data[14]);
  g_status.imuEnabled = data[16];
  g_status.imuValid = data[17];

  // 闂佽崵鍠愰悷杈╁緤妤ｅ啯鍊甸柣锝呯灱绾句粙鏌″搴′簽闁搞劍濞婇弻娑㈡晲閸愩劌顬夐梺鍝勬閸犳牠鐛?(闂備礁鎼崐鐟邦熆濮椻偓璺?
  if (len >= 26) {
    for (int i = 0; i < 4; i++) {
      g_status.encVel[i] = (int16_t)getU16LE(&data[18 + i * 2]);
    }
  } else {
    for (int i = 0; i < 4; i++) g_status.encVel[i] = 0;
  }

  g_status.lastRxMs = millis();

  buildStatusJson();
  return true;
}

static bool sendFrame(uint8_t msgType, uint8_t flags, uint8_t seq, const uint8_t *payload, uint16_t payloadLen) {
  uint8_t frame[FRAME_MAX];
  uint8_t enc[COBS_MAX];

  uint16_t frameLen = (uint16_t)(1 + 1 + 1 + 1 + 1 + 2 + payloadLen + 2 + 1);
  if (frameLen > sizeof(frame)) return false;

  frame[0] = PROTO_SOF;
  frame[1] = PROTO_VER;
  frame[2] = msgType;
  frame[3] = flags;
  frame[4] = seq;
  putU16LE(&frame[5], payloadLen);
  if (payloadLen > 0 && payload) memcpy(&frame[7], payload, payloadLen);

  uint16_t crc = crc16CcittFalse(&frame[1], (uint16_t)(6 + payloadLen));
  putU16LE(&frame[7 + payloadLen], crc);
  frame[9 + payloadLen] = PROTO_EOF;

  uint16_t encLen = cobsEncode(frame, frameLen, enc, (uint16_t)(sizeof(enc) - 1));
  if (encLen == 0) return false;
  enc[encLen++] = 0x00;
  uint16_t off = 0;
  uint32_t t0 = millis();
  while (off < encLen) {
    int avail = Serial.availableForWrite();
    if (avail > 0) {
      uint16_t n = (uint16_t)avail;
      if (n > (uint16_t)(encLen - off)) n = (uint16_t)(encLen - off);
      size_t written = Serial.write(enc + off, n);
      if (written == 0) {
        if ((uint32_t)(millis() - t0) > UART_WRITE_TIMEOUT_MS) {
          g_uartTxTimeouts++;
          return false;
        }
      } else {
        off = (uint16_t)(off + written);
      }
      ESP.wdtFeed();
      yield();
      continue;
    }
    if ((uint32_t)(millis() - t0) > UART_WRITE_TIMEOUT_MS) {
      g_uartTxTimeouts++;
      return false;
    }
    ESP.wdtFeed();
    yield();
  }
  return true;
}

static void processDecodedFrame(const uint8_t *frame, uint16_t len) {
  if (len < 10) return;
  if (frame[0] != PROTO_SOF || frame[len - 1] != PROTO_EOF) return;

  uint8_t ver = frame[1];
  uint8_t msgType = frame[2];
  uint8_t flags = frame[3];
  uint8_t seq = frame[4];
  uint16_t payloadLen = getU16LE(&frame[5]);

  if (ver != PROTO_VER) return;
  if ((uint16_t)(payloadLen + 10) != len) return;

  uint16_t recvCrc = getU16LE(&frame[7 + payloadLen]);
  uint16_t calcCrc = crc16CcittFalse(&frame[1], (uint16_t)(6 + payloadLen));
  if (recvCrc != calcCrc) return;

  const uint8_t *payload = &frame[7];

  if (msgType == MSG_ACK || msgType == MSG_NACK) {
    if (payloadLen < 4) return;

    g_ack.ready = true;
    g_ack.nack = (msgType == MSG_NACK) || ((flags & FLAG_IS_NACK) != 0);
    g_ack.seq = payload[0];
    g_ack.err = getU16LE(&payload[1]);
    g_ack.detail = payload[3];

    uint16_t extraLen = (uint16_t)(payloadLen - 4);
    if (extraLen > sizeof(g_ack.extra)) extraLen = sizeof(g_ack.extra);
    g_ack.extraLen = extraLen;
    if (extraLen > 0) memcpy(g_ack.extra, &payload[4], extraLen);

    if (!g_ack.nack && g_ack.err == PROTO_ERR_OK && g_ack.extraLen >= 26) {
      parseStatusExtra(g_ack.extra, g_ack.extraLen);
    }
  }
}

static void pumpSerial() {
  uint16_t spin = 0;
  uint16_t budget = SERIAL_RX_BUDGET_BYTES;
  while (budget-- > 0 && Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();
    ESP.wdtFeed();
    if (((++spin) & 0x3Fu) == 0u) {
      yield();
    }
    if (b == 0x00) {
      if (g_rxAccumLen > 0) {
        uint8_t dec[FRAME_MAX];
        uint16_t decLen = cobsDecode(g_rxAccum, g_rxAccumLen, dec, sizeof(dec));
        if (decLen > 0) processDecodedFrame(dec, decLen);
      }
      g_rxAccumLen = 0;
      continue;
    }

    if (g_rxAccumLen >= sizeof(g_rxAccum)) {
      g_rxAccumLen = 0;
      continue;
    }

    g_rxAccum[g_rxAccumLen++] = b;
  }
}

static bool sendCommandAwaitAck(uint8_t msgType,
                                const uint8_t *payload,
                                uint16_t payloadLen,
                                uint16_t timeoutMs,
                                uint16_t *errOut,
                                uint8_t *detailOut) {
  uint8_t seq = nextSeq();
  clearAckState();

  if (!sendFrame(msgType, FLAG_ACK_REQ, seq, payload, payloadLen)) {
    if (errOut) *errOut = 0x0401;
    if (detailOut) *detailOut = 0;
    return false;
  }

  uint32_t start = millis();
  while ((uint32_t)(millis() - start) < timeoutMs) {
    ESP.wdtFeed();
    pumpSerial();
    if (g_ack.ready && g_ack.seq == seq) {
      if (errOut) *errOut = g_ack.err;
      if (detailOut) *detailOut = g_ack.detail;
      return (!g_ack.nack && g_ack.err == PROTO_ERR_OK);
    }
    delay(1);
  }

  if (errOut) *errOut = 0xFFFF;
  if (detailOut) *detailOut = 0;
  return false;
}

static bool sendDriveNoAck(float v, float w) {
  uint8_t seq = nextSeq();
  v = clampf(v, -1.0f, 1.0f);
  w = clampf(w, -1.0f, 1.0f);
  int16_t vQ15 = (int16_t)(v * 32767.0f);
  int16_t wQ15 = (int16_t)(w * 32767.0f);
  uint8_t payload[4];
  putU16LE(&payload[0], (uint16_t)vQ15);
  putU16LE(&payload[2], (uint16_t)wQ15);
  return sendFrame(MSG_CMD_SET_DRIVE, 0, seq, payload, sizeof(payload));
}

static bool sendModeNoAck(uint8_t mode) {
  uint8_t seq = nextSeq();
  uint8_t payload[1] = {mode};
  return sendFrame(MSG_CMD_SET_MODE, 0, seq, payload, sizeof(payload));
}

static bool requestStatusNoBlock(void) {
  uint8_t seq = nextSeq();
  return sendFrame(MSG_CMD_GET_STATUS, FLAG_ACK_REQ, seq, nullptr, 0);
}

static void applyPendingStopIfIdle(uint32_t now) {
  if ((uint32_t)(now - g_lastHttpCmdMs) > CMD_IDLE_STOP_MS) {
    g_pendingCmdV = 0.0f;
    g_pendingCmdW = 0.0f;
    g_cmdDirty = true;
  }
}

static void serviceDriveTx(uint32_t now) {
  static uint32_t lastDriveTx = 0;
  if ((uint32_t)(now - lastDriveTx) < DRIVE_SEND_PERIOD_MS) return;

  // If command has not changed, keepalive at a lower rate to reduce link pressure.
  if (!g_cmdDirty && (uint32_t)(now - g_lastCmdTxMs) < 150u) {
    return;
  }

  if (g_cmdDirty) {
    g_cmdDirty = false;
  }

  // 闂備胶鍎甸弲娑㈡偤閵娧勬殰閻庢稒蓱婵挳鎮归幁鎺戝闁哄棗绻橀弻娑樜熼悡搴㈢€诲銈庡亝閸旀瑥鐣烽妸鈺佺闁圭儤鎸歌闂備胶顭堥敃锕傚Υ鐎ｎ剚顫曟繝闈涙川閳绘梻鈧箍鍎遍幊宥囧垝閸偒娈介柣鎰煐閻濐亞绱掗鍡涙鐎垫澘瀚板畷鐔碱敇閻斿皝鍋?
  // Keep TX command aligned with latest pending command.
  g_txCmdV = g_pendingCmdV;
  g_txCmdW = g_pendingCmdW;

  if (sendDriveNoAck(g_txCmdV, g_txCmdW)) {
    g_lastCmdV = g_txCmdV;
    g_lastCmdW = g_txCmdW;
    g_lastCmdTxMs = now;
  }
  lastDriveTx = now;
}

static void serviceStatusPoll(uint32_t now) {
  static uint32_t lastPoll = 0;
  if ((uint32_t)(now - lastPoll) < STATUS_POLL_MS) return;
  lastPoll = now;
  (void)requestStatusNoBlock();
}

static void serviceStaConnectState(uint32_t now) {
  if (!g_sta_connecting) return;
  wl_status_t st = WiFi.status();
  if (st == WL_CONNECTED || (uint32_t)(now - g_sta_connect_start_ms) > STA_CONNECT_TIMEOUT_MS) {
    g_sta_connecting = false;
  }
}

// -------------------- Web UI --------------------
// -------------------- Web UI --------------------
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ESP01S 机器人控制台</title>
  <style>
    :root{
      --bg:#eef3f9;
      --card:#ffffff;
      --line:#d8e1ee;
      --text:#1e293b;
      --muted:#64748b;
      --primary:#0f766e;
      --primary2:#0ea5a4;
      --danger:#c81e1e;
      --ok:#15803d;
      --warn:#b45309;
    }
    *{ box-sizing:border-box; }
    body{
      font-family: "Segoe UI", "Microsoft YaHei", sans-serif;
      margin:0;
      color:var(--text);
      background:
        radial-gradient(1200px 420px at 80% -10%, #d7f3ef 0%, transparent 60%),
        radial-gradient(900px 320px at -20% 120%, #dbe8ff 0%, transparent 60%),
        var(--bg);
    }
    .wrap{ max-width:760px; margin:0 auto; padding:14px; }
    .head{ margin:0 0 10px; font-size:20px; font-weight:700; letter-spacing:0.2px; }
    .sub{ color:var(--muted); font-size:13px; margin:0 0 10px; }
    .card{
      background:var(--card);
      border:1px solid var(--line);
      border-radius:14px;
      padding:12px;
      margin-bottom:12px;
      box-shadow:0 6px 18px rgba(15,23,42,0.04);
    }
    .title{ margin:0 0 10px; font-size:17px; font-weight:700; }
    .row{ display:flex; justify-content:space-between; gap:10px; margin:8px 0; font-size:14px; align-items:center; }
    .label{ color:var(--muted); }
    .value{ font-weight:700; background:#f3f7fc; border:1px solid #e2e8f0; border-radius:999px; padding:2px 10px; font-size:13px; }
    .ok{ color:var(--ok); }
    .warn{ color:var(--warn); }
    .bad{ color:var(--danger); }
    .dpad{ display:grid; grid-template-columns:repeat(3, 1fr); gap:8px; max-width:340px; margin:8px auto; }
    .hint{ font-size:12px; color:var(--muted); text-align:center; }
    button{
      border:0;
      border-radius:10px;
      min-height:54px;
      font-size:18px;
      color:#fff;
      background:linear-gradient(180deg, var(--primary2), var(--primary));
      transition:transform .04s ease, opacity .15s ease, box-shadow .15s ease;
      box-shadow:0 6px 14px rgba(15,118,110,.22);
      touch-action:manipulation;
    }
    button:active{ transform:scale(0.98); }
    .stop{ background:linear-gradient(180deg, #ef4444, var(--danger)); font-size:14px; box-shadow:0 6px 14px rgba(200,30,30,.25); }
    .modes{ display:grid; grid-template-columns:repeat(3, 1fr); gap:8px; }
    .mode{ min-height:40px; font-size:14px; }
    .mode.active{ outline:2px solid #14b8a6; outline-offset:1px; }
    input{ min-height:38px; padding:0 10px; border:1px solid #cbd5e1; border-radius:8px; font-size:14px; background:#fff; }
    pre{ background:#0f172a; color:#e5e7eb; border-radius:10px; padding:10px; overflow:auto; max-height:180px; font-size:12px; line-height:1.35; }
  </style>
</head>
<body>
  <div class="wrap">
    <h1 class="head">ESP01S 机器人控制台</h1>
    <div class="sub">方向遥控 + 模式切换 + 状态监控 + AP/STA 联网</div>
    <div class="card">
      <h2 class="title">运动控制</h2>
      <div class="row"><span class="label">当前动作</span><b id="action" class="value">停止</b></div>
      <div class="row"><span class="label">当前指令</span><b id="cmd" class="value">v=0.00 w=0.00</b></div>
      <div class="dpad">
        <div></div><button id="up">↑</button><div></div>
        <button id="left">←</button><button id="stop" class="stop">STOP</button><button id="right">→</button>
        <div></div><button id="down">↓</button><div></div>
      </div>
      <div class="hint">长按方向键持续控制，松手自动停止</div>
    </div>

    <div class="card">
      <h2 class="title">模式切换</h2>
      <div class="modes">
        <button id="mode0" class="mode" onclick="setMode(0)">待机</button>
        <button id="mode1" class="mode" onclick="setMode(1)">开环</button>
        <button id="mode2" class="mode" onclick="setMode(2)">闭环</button>
      </div>
    </div>

    <div class="card">
      <h2 class="title">状态</h2>
      <div class="row"><span class="label">模式</span><b id="mode" class="value">--</b></div>
      <div class="row"><span class="label">来源</span><b id="src" class="value">--</b></div>
      <div class="row"><span class="label">电池</span><b id="batt" class="value">--</b></div>
      <div class="row"><span class="label">最近更新</span><b id="ts" class="value">--</b></div>
      <pre id="status">{}</pre>
    </div>

    <div class="card">
      <h2 class="title">Wi-Fi 联网（AP+STA）</h2>
      <div class="row"><span class="label">当前 AP</span><b id="apInfo" class="value">ESP01S_CTRL</b></div>
      <div class="row"><span class="label">STA 状态</span><b id="staState" class="value">--</b></div>
      <div class="row"><span class="label">STA IP</span><b id="staIp" class="value">--</b></div>
      <div class="row"><span class="label">信道/终端</span><b id="chInfo" class="value">--</b></div>
      <div class="hint" id="wifiHint">提示：当 STA 成功联网后，AP 可能切到同信道，手机可能短暂重连。</div>
      <div style="display:grid;grid-template-columns:1fr;gap:8px;">
        <input id="wifiSsid" placeholder="路由器 SSID">
        <input id="wifiPass" placeholder="路由器密码" type="password">
        <button onclick="connectWifi()" style="min-height:40px;font-size:14px;">连接路由器</button>
      </div>
    </div>
  </div>

<script>
const V = 0.78;
const W = 0.52;
const HOLD_KEEP_MS = 160;
const REQ_TIMEOUT_MS = 900;
let inflight = false;
let queued = null;
let lastSentV = 99, lastSentW = 99;
let lastSentMs = 0;
let holdV = 0, holdW = 0;
let holdActive = false;
let holdTimer = null;
let wifiPollDiv = 0;

function fmt(x){ return (Math.round(x*100)/100).toFixed(2); }
function action(v,w){
  if(Math.abs(v)<0.01&&Math.abs(w)<0.01)return '停止';
  if(Math.abs(v)>=Math.abs(w))return v>0?'前进':'后退';
  return w>0?'左转':'右转';
}

function doSend(v,w){
  inflight = true;
  const ctl = new AbortController();
  const to = setTimeout(()=>ctl.abort(), REQ_TIMEOUT_MS);
  fetch(`/cmd?v=${v.toFixed(3)}&w=${w.toFixed(3)}`, {cache:'no-store', signal: ctl.signal})
    .catch(()=>{})
    .finally(()=>{
      clearTimeout(to);
      inflight = false;
      if(queued){
        const q = queued;
        queued = null;
        doSend(q.v, q.w);
      }
    });
}

function send(v,w){
  document.getElementById('cmd').textContent = `v=${fmt(v)} w=${fmt(w)}`;
  document.getElementById('action').textContent = action(v,w);
  const now = Date.now();
  const dv = Math.abs(v - lastSentV);
  const dw = Math.abs(w - lastSentW);
  if(dv < 0.01 && dw < 0.01 && (now - lastSentMs) < 50) return;
  lastSentV = v; lastSentW = w; lastSentMs = now;
  if(inflight){
    queued = {v,w};
    return;
  }
  doSend(v,w);
}

function startHold(v,w){
  if (holdActive && Math.abs(holdV - v) < 0.001 && Math.abs(holdW - w) < 0.001) return;
  holdV = v; holdW = w;
  holdActive = true;
  send(holdV, holdW);
  if (holdTimer) clearInterval(holdTimer);
  holdTimer = setInterval(()=>{
    if (holdActive) send(holdV, holdW);
  }, HOLD_KEEP_MS);
}

function stopHold(sendStop=true){
  holdActive = false;
  if (holdTimer) { clearInterval(holdTimer); holdTimer = null; }
  if(sendStop) send(0,0);
}

function bindHold(id, v, w){
  const e = document.getElementById(id);
  const down = (ev) => {
    ev.preventDefault();
    if (e.setPointerCapture && ev.pointerId !== undefined) e.setPointerCapture(ev.pointerId);
    startHold(v, w);
  };
  const up = (ev) => { ev.preventDefault(); stopHold(true); };
  e.addEventListener('pointerdown', down);
  e.addEventListener('pointerup', up);
  e.addEventListener('pointercancel', up);
  e.addEventListener('contextmenu', ev => ev.preventDefault());
}

bindHold('up', V, 0);
bindHold('down', -V, 0);
bindHold('left', 0, W);
bindHold('right', 0, -W);
document.getElementById('stop').addEventListener('click', ()=>stopHold(true));
window.addEventListener('blur', ()=>stopHold(true));
document.addEventListener('visibilitychange', ()=>{ if(document.hidden) stopHold(true); });

function setModeActive(m){
  [0,1,2].forEach(i=>{
    const el = document.getElementById(`mode${i}`);
    if(!el) return;
    if(i===m) el.classList.add('active');
    else el.classList.remove('active');
  });
}

async function setMode(m){ await fetch(`/mode?m=${m}`).catch(()=>{}); }

async function connectWifi(){
  const ssid = document.getElementById('wifiSsid').value.trim();
  const pass = document.getElementById('wifiPass').value;
  if(!ssid){ alert('请填写 SSID'); return; }
  document.getElementById('staState').textContent = 'CONNECTING (连接中)';
  document.getElementById('staState').className = 'value warn';
  const body = `ssid=${encodeURIComponent(ssid)}&pass=${encodeURIComponent(pass)}`;
  await fetch('/wifi_set', {
    method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body
  }).catch(()=>{});
}

async function poll(){
  try{
    const t=await (await fetch('/status',{cache:'no-store'})).text();
    document.getElementById('status').textContent=t;
    document.getElementById('ts').textContent=new Date().toLocaleTimeString('zh-CN',{hour12:false});
    const j=JSON.parse(t);
    const mm={0:'待机',1:'开环',2:'闭环'};
    const ss={0:'无',1:'手柄',2:'ESP',3:'PC'};
    document.getElementById('mode').textContent=mm[j.mode]??j.mode;
    setModeActive(Number(j.mode));
    document.getElementById('src').textContent=ss[j.src]??j.src;
    if(j.vbatt!==undefined && j.percent!==undefined){
      document.getElementById('batt').textContent=`${j.vbatt.toFixed(2)}V ${j.percent.toFixed(1)}%`;
    }
  }catch(_){ }
  if ((wifiPollDiv++ % 3) === 0) {
    try{
      const w=await (await fetch('/wifi_status',{cache:'no-store'})).json();
      document.getElementById('apInfo').textContent = `${w.ap_ssid} (${w.ap_ip})`;
      document.getElementById('staState').textContent = `${w.sta_status} ${w.sta_connected?'(已连接)':'(未连接)'}`;
      document.getElementById('staState').className = `value ${w.sta_connected?'ok':'warn'}`;
      document.getElementById('staIp').textContent = w.sta_ip || '--';
      document.getElementById('chInfo').textContent = `CH${w.channel} / ${w.ap_clients}台`;
      document.getElementById('wifiHint').textContent = w.sta_connected
        ? '提示：已联网。请在同一局域网通过 STA IP 访问。'
        : '提示：当 STA 成功联网后，AP 可能切到同信道，手机可能短暂重连。';
    }catch(_){ }
  }
  setTimeout(poll, 1500);
}
poll();
</script>
</body>
</html>
)HTML";

// -------------------- HTTP Handlers --------------------
static void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

static void handleCmd() {
  if (!server.hasArg("v") || !server.hasArg("w")) {
    server.send(400, "text/plain", "missing v/w");
    return;
  }

  float v = clampf(server.arg("v").toFloat(), -1.0f, 1.0f);
  float w = clampf(server.arg("w").toFloat(), -1.0f, 1.0f);
  uint32_t now = millis();

  if (fabsf(v - g_lastHttpCmdV) < 0.001f &&
      fabsf(w - g_lastHttpCmdW) < 0.001f &&
      (uint32_t)(now - g_lastHttpRxMs) < 120u) {
    server.send(200, "text/plain", "OK");
    return;
  }
  g_lastHttpCmdV = v;
  g_lastHttpCmdW = w;
  g_lastHttpRxMs = now;

  // Drop duplicate commands in a short window to reduce Wi-Fi/UART load.
  if (fabsf(v - g_lastCmdV) < 0.001f &&
      fabsf(w - g_lastCmdW) < 0.001f &&
      (uint32_t)(now - g_lastCmdTxMs) < CMD_DUP_FILTER_MS) {
    server.send(200, "text/plain", "OK");
    return;
  }
  g_pendingCmdV = v;
  g_pendingCmdW = w;
  g_lastHttpCmdMs = now;
  g_cmdDirty = true;
  server.send(200, "text/plain", "OK");
}

static void handleMode() {
  if (!server.hasArg("m")) {
    server.send(400, "text/plain", "missing m");
    return;
  }

  int m = server.arg("m").toInt();
  if (m < 0 || m > 2) {
    server.send(400, "text/plain", "invalid m");
    return;
  }

  bool ok = sendModeNoAck((uint8_t)m);
  if (ok) server.send(200, "application/json", "{\"ok\":true}");
  else server.send(500, "application/json", "{\"ok\":false}");
}

static void handleStatus() {
  server.send(200, "application/json", g_statusJson);
}

static void handleHealth() {
  char buf[520];
  uint32_t age = (g_status.lastRxMs == 0) ? 0xFFFFFFFFu : (millis() - g_status.lastRxMs);
  snprintf(buf, sizeof(buf),
           "{\"last_cmd_v\":%.3f,\"last_cmd_w\":%.3f,\"last_ack_ready\":%s,"
           "\"last_ack_nack\":%s,\"last_ack_err\":%u,\"status_age_ms\":%lu,"
           "\"reset_reason\":\"%s\",\"free_heap\":%lu,"
           "\"uart_tx_timeouts\":%lu,\"loop_max_gap_ms\":%lu,\"uptime_ms\":%lu}",
           g_lastCmdV,
           g_lastCmdW,
           g_ack.ready ? "true" : "false",
           g_ack.nack ? "true" : "false",
           (unsigned)g_ack.err,
           (unsigned long)age,
           g_resetReason,
           (unsigned long)ESP.getFreeHeap(),
           (unsigned long)g_uartTxTimeouts,
           (unsigned long)g_loopMaxGapMs,
           (unsigned long)millis());
  server.send(200, "application/json", buf);
}

static const char* staStatusText(wl_status_t st) {
  switch (st) {
    case WL_IDLE_STATUS: return "IDLE";
    case WL_NO_SSID_AVAIL: return "NO_SSID";
    case WL_SCAN_COMPLETED: return "SCAN_DONE";
    case WL_CONNECTED: return "CONNECTED";
    case WL_CONNECT_FAILED: return "CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "LOST";
    case WL_DISCONNECTED: return "DISCONNECTED";
    default: return "UNKNOWN";
  }
}

static void startStaConnect(const char *ssid, const char *pass) {
  if (!ssid || ssid[0] == '\0') return;
  strncpy(g_staTargetSsid, ssid, sizeof(g_staTargetSsid) - 1);
  g_staTargetSsid[sizeof(g_staTargetSsid) - 1] = '\0';
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, pass);
  g_sta_connecting = true;
  g_sta_connect_start_ms = millis();
}

static void handleWifiSet() {
  if (!server.hasArg("ssid")) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"missing ssid\"}");
    return;
  }
  const String ssidArg = server.arg("ssid");
  const String passArg = server.hasArg("pass") ? server.arg("pass") : String("");
  char ssid[33] = {0};
  char pass[65] = {0};
  ssidArg.toCharArray(ssid, sizeof(ssid));
  passArg.toCharArray(pass, sizeof(pass));
  if (ssid[0] == '\0' || strlen(ssid) > 32 || strlen(pass) > 64) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"invalid ssid/pass\"}");
    return;
  }
  (void)saveWifiCfg(ssid, pass);
  startStaConnect(ssid, pass);
  server.send(200, "application/json", "{\"ok\":true}");
}

static void handleWifiStatus() {
  char buf[420];
  IPAddress apIp = WiFi.softAPIP();
  IPAddress staIp = WiFi.localIP();
  wl_status_t st = WiFi.status();
  snprintf(buf, sizeof(buf),
           "{\"ok\":true,\"ap_ssid\":\"%s\",\"ap_ip\":\"%u.%u.%u.%u\","
           "\"sta_ssid\":\"%s\",\"sta_status\":\"%s\",\"sta_connected\":%s,"
           "\"sta_ip\":\"%u.%u.%u.%u\",\"rssi\":%d,\"channel\":%u,\"ap_clients\":%u}",
           AP_SSID,
           apIp[0], apIp[1], apIp[2], apIp[3],
           g_staTargetSsid,
           staStatusText(st),
           (st == WL_CONNECTED) ? "true" : "false",
           staIp[0], staIp[1], staIp[2], staIp[3],
           (st == WL_CONNECTED) ? WiFi.RSSI() : 0,
           (unsigned)WiFi.channel(),
           (unsigned)WiFi.softAPgetStationNum());
  server.send(200, "application/json", buf);
}

static void initWifiStack() {
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setOutputPower(8.0f);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
}

static void tryAutoConnectSta() {
  wifi_cfg_t cfg;
  if (loadWifiCfg(&cfg)) {
    startStaConnect(cfg.ssid, cfg.pass);
  }
}

static void registerHttpRoutes() {
  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.on("/mode", handleMode);
  server.on("/status", handleStatus);
  server.on("/health", handleHealth);
  server.on("/wifi_set", HTTP_POST, handleWifiSet);
  server.on("/wifi_status", handleWifiStatus);
}

void setup() {
  Serial.begin(115200);
  delay(80);
  ESP.wdtEnable(WDT_TIMEOUT_MS);
  ESP.wdtFeed();
  EEPROM.begin(sizeof(wifi_cfg_t));
  strncpy(g_resetReason, ESP.getResetReason().c_str(), sizeof(g_resetReason) - 1);
  g_resetReason[sizeof(g_resetReason) - 1] = '\0';
  g_lastHttpCmdMs = millis();

  initWifiStack();
  tryAutoConnectSta();
  registerHttpRoutes();
  server.begin();
}

void loop() {
  ESP.wdtFeed();
  server.handleClient();
  pumpSerial();
  yield();

  uint32_t now = millis();
  if (g_loopLastMs != 0u) {
    uint32_t gap = (uint32_t)(now - g_loopLastMs);
    if (gap > g_loopMaxGapMs) g_loopMaxGapMs = gap;
  }
  g_loopLastMs = now;
  applyPendingStopIfIdle(now);
  serviceDriveTx(now);
  serviceStatusPoll(now);
  serviceStaConnectState(now);
}
