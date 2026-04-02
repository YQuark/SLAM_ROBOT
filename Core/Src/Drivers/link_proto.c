#include "link_proto.h"

#include <string.h>

#include "robot_config.h"
#include "robot_control.h"
#include "encoder.h"

#ifndef LINK_PROTO_VER
#define LINK_PROTO_VER 0x01u
#endif

#ifndef LINK_PAYLOAD_MAX
#define LINK_PAYLOAD_MAX 192u
#endif

#ifndef LINK_RX_ACCUM_MAX
#define LINK_RX_ACCUM_MAX 260u
#endif

#define LINK_SOF 0xA5u
#define LINK_EOF 0x5Au
#define LINK_FRAME_MIN 10u
#define LINK_RESP_MAX 220u
#define LINK_COBS_MAX 300u
#define LINK_TX_QUEUE_DEPTH 4u

typedef struct {
    UART_HandleTypeDef *huart;

    uint8_t rx_accum[LINK_RX_ACCUM_MAX];
    uint16_t rx_accum_len;

    uint8_t rx_packet[LINK_RX_ACCUM_MAX];
    uint16_t rx_packet_len;
    volatile uint8_t rx_packet_ready;

    uint8_t last_req_valid;
    uint8_t last_req_seq;
    uint8_t last_req_type;

    uint8_t last_resp_valid;
    uint8_t last_resp_seq;
    uint8_t last_resp_type;
    uint16_t last_resp_len;
    uint8_t last_resp_data[LINK_COBS_MAX];

    volatile uint8_t tx_busy;
    volatile uint8_t tx_head;
    volatile uint8_t tx_tail;
    volatile uint8_t tx_count;
    uint16_t tx_len[LINK_TX_QUEUE_DEPTH];
    uint8_t tx_data[LINK_TX_QUEUE_DEPTH][LINK_COBS_MAX];
} link_ctx_t;

static link_ctx_t s_ctx[LINK_ID_MAX];
static uint8_t s_diag_inited = 0u;

static uint8_t valid_link(link_id_t link_id)
{
    return (link_id >= 0 && link_id < LINK_ID_MAX) ? 1u : 0u;
}

static uint16_t crc16_ccitt_false(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    for (i = 0; i < len; ++i) {
        uint8_t j;
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (j = 0; j < 8; ++j) {
            if (crc & 0x8000u) crc = (uint16_t)((crc << 1) ^ 0x1021u);
            else crc <<= 1;
        }
    }
    return crc;
}

static uint16_t cobs_encode(const uint8_t *input, uint16_t len, uint8_t *output, uint16_t out_max)
{
    uint16_t read_index = 0;
    uint16_t write_index = 1;
    uint16_t code_index = 0;
    uint8_t code = 1;

    if (out_max == 0u) return 0u;

    while (read_index < len) {
        if (write_index >= out_max) return 0u;

        if (input[read_index] == 0u) {
            output[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        } else {
            output[write_index++] = input[read_index++];
            code++;
            if (code == 0xFFu) {
                output[code_index] = code;
                code = 1;
                code_index = write_index++;
                if (write_index > out_max) return 0u;
            }
        }
    }

    if (code_index >= out_max) return 0u;
    output[code_index] = code;
    return write_index;
}

static uint16_t cobs_decode(const uint8_t *input, uint16_t len, uint8_t *output, uint16_t out_max)
{
    uint16_t read_index = 0;
    uint16_t write_index = 0;

    while (read_index < len) {
        uint8_t code = input[read_index++];
        uint8_t i;

        if (code == 0u) return 0u;

        for (i = 1; i < code; ++i) {
            if (read_index >= len || write_index >= out_max) return 0u;
            output[write_index++] = input[read_index++];
        }

        if (code != 0xFFu && read_index < len) {
            if (write_index >= out_max) return 0u;
            output[write_index++] = 0u;
        }
    }

    return write_index;
}

static void put_u16le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void put_u32le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint16_t get_u16le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t clamp_i16_from_f(float x)
{
    if (x > 32767.0f) return 32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)x;
}

static HAL_StatusTypeDef link_send_encoded(link_ctx_t *ctx,
                                           const uint8_t *data,
                                           uint16_t len)
{
    if (!ctx || !ctx->huart || !data || len == 0u) return HAL_ERROR;
    if (len > LINK_COBS_MAX) return HAL_ERROR;

    __disable_irq();
    if (ctx->tx_count >= LINK_TX_QUEUE_DEPTH) {
        __enable_irq();
        return HAL_BUSY;
    }
    memcpy(ctx->tx_data[ctx->tx_tail], data, len);
    ctx->tx_len[ctx->tx_tail] = len;
    ctx->tx_tail = (uint8_t)((ctx->tx_tail + 1u) % LINK_TX_QUEUE_DEPTH);
    ctx->tx_count++;
    __enable_irq();
    return HAL_OK;
}

static void link_kick_tx(link_ctx_t *ctx)
{
    HAL_StatusTypeDef st;
    uint8_t head;
    uint16_t len;

    if (!ctx || !ctx->huart) return;

    __disable_irq();
    if (ctx->tx_busy || ctx->tx_count == 0u || ctx->huart->gState != HAL_UART_STATE_READY) {
        __enable_irq();
        return;
    }
    head = ctx->tx_head;
    len = ctx->tx_len[head];
    ctx->tx_busy = 1u;
    __enable_irq();

    st = HAL_UART_Transmit_IT(ctx->huart, ctx->tx_data[head], len);
    if (st != HAL_OK) {
        __disable_irq();
        ctx->tx_busy = 0u;
        __enable_irq();
    }
}

static HAL_StatusTypeDef link_send_frame(link_id_t link_id,
                                         uint8_t msg_type,
                                         uint8_t flags,
                                         uint8_t seq,
                                         const uint8_t *payload,
                                         uint16_t payload_len,
                                         uint8_t cache_response)
{
    link_ctx_t *ctx;
    uint8_t frame[LINK_RESP_MAX];
    uint8_t encoded[LINK_COBS_MAX];
    uint16_t frame_len;
    uint16_t enc_len;
    uint16_t crc;
    HAL_StatusTypeDef st;

    if (!valid_link(link_id)) return HAL_ERROR;
    if (payload_len > LINK_PAYLOAD_MAX) return HAL_ERROR;

    ctx = &s_ctx[link_id];
    frame_len = (uint16_t)(1u + 1u + 1u + 1u + 1u + 2u + payload_len + 2u + 1u);
    if (frame_len > sizeof(frame)) return HAL_ERROR;

    frame[0] = LINK_SOF;
    frame[1] = LINK_PROTO_VER;
    frame[2] = msg_type;
    frame[3] = flags;
    frame[4] = seq;
    put_u16le(&frame[5], payload_len);
    if (payload_len > 0u && payload) memcpy(&frame[7], payload, payload_len);

    crc = crc16_ccitt_false(&frame[1], (uint16_t)(6u + payload_len));
    put_u16le(&frame[7 + payload_len], crc);
    frame[9 + payload_len] = LINK_EOF;

    enc_len = cobs_encode(frame, frame_len, encoded, (uint16_t)(sizeof(encoded) - 1u));
    if (enc_len == 0u) return HAL_ERROR;
    encoded[enc_len++] = 0x00u;

    st = link_send_encoded(ctx, encoded, enc_len);
    if (st != HAL_OK) {
        LinkDiag_IncTxFail(link_id);
        LinkDiag_Count(link_id, LINK_ERR_UART_IO);
        LinkDiag_PushFault(link_id, FAULT_SEV_ERROR, LINK_ERR_UART_IO, seq, msg_type, 0u, 0u);
        return st;
    }
    link_kick_tx(ctx);

    if (cache_response) {
        ctx->last_resp_valid = 1u;
        ctx->last_resp_seq = seq;
        ctx->last_resp_type = msg_type;
        ctx->last_resp_len = enc_len;
        if (enc_len <= sizeof(ctx->last_resp_data)) {
            memcpy(ctx->last_resp_data, encoded, enc_len);
        } else {
            ctx->last_resp_valid = 0u;
        }
    }

    return HAL_OK;
}

static void append_status_payload(uint8_t *out, uint16_t *out_len,
                                  const BatteryStatus_t *batt,
                                  const MPU6050_Data_t *imu)
{
    const RobotControlState_t *st = RobotControl_GetState();
    const Attitude_t *att = Attitude_GetData();
    uint16_t p = 0;
    int16_t v_q15;
    int16_t w_q15;
    int16_t v_est_q15;
    int16_t w_est_q15;
    int16_t yaw_est_x100;
    int16_t raw_left_q15;
    int16_t raw_right_q15;
    uint16_t vb_mv = 0u;
    uint16_t pct_x10 = 0u;
    int16_t gz_x10 = 0;
    int16_t raw_ax_mg = 0;
    int16_t raw_ay_mg = 0;
    int16_t raw_az_mg = 0;

    if (batt) {
        float vb = batt->voltage * 1000.0f;
        float pct = batt->percent * 10.0f;
        if (vb < 0.0f) vb = 0.0f;
        if (vb > 65535.0f) vb = 65535.0f;
        if (pct < 0.0f) pct = 0.0f;
        if (pct > 1000.0f) pct = 1000.0f;
        vb_mv = (uint16_t)(vb + 0.5f);
        pct_x10 = (uint16_t)(pct + 0.5f);
    }

    if (imu) {
        float gz = imu->gz_dps * 10.0f;
        float ax = imu->ax_g * 1000.0f;
        float ay = imu->ay_g * 1000.0f;
        float az = imu->az_g * 1000.0f;
        if (att && att->valid) {
            gz = (imu->gz_dps - att->gyro_bias_z) * 10.0f;
        }
        if (gz > 32767.0f) gz = 32767.0f;
        if (gz < -32768.0f) gz = -32768.0f;
        if (ax > 32767.0f) ax = 32767.0f;
        if (ax < -32768.0f) ax = -32768.0f;
        if (ay > 32767.0f) ay = 32767.0f;
        if (ay < -32768.0f) ay = -32768.0f;
        if (az > 32767.0f) az = 32767.0f;
        if (az < -32768.0f) az = -32768.0f;
        gz_x10 = (int16_t)gz;
        raw_ax_mg = (int16_t)ax;
        raw_ay_mg = (int16_t)ay;
        raw_az_mg = (int16_t)az;
    }

    v_q15 = clamp_i16_from_f(st->v_cmd * 32767.0f);
    w_q15 = clamp_i16_from_f(st->w_cmd * 32767.0f);
    raw_left_q15 = clamp_i16_from_f(st->raw_left_cmd * 32767.0f);
    raw_right_q15 = clamp_i16_from_f(st->raw_right_cmd * 32767.0f);
    v_est_q15 = clamp_i16_from_f(st->v_est * 32767.0f);
    w_est_q15 = clamp_i16_from_f(st->w_est * 32767.0f);
    yaw_est_x100 = clamp_i16_from_f(st->yaw_est * 100.0f);

    put_u32le(&out[p], HAL_GetTick()); p += 4;
    out[p++] = (uint8_t)RobotControl_GetMode();
    out[p++] = (uint8_t)st->src;
    put_u16le(&out[p], vb_mv); p += 2;
    put_u16le(&out[p], pct_x10); p += 2;
    put_u16le(&out[p], (uint16_t)v_q15); p += 2;
    put_u16le(&out[p], (uint16_t)w_q15); p += 2;
    put_u16le(&out[p], (uint16_t)gz_x10); p += 2;
    out[p++] = st->imu_enabled;
    out[p++] = st->imu_valid;
    put_u16le(&out[p], (uint16_t)v_est_q15); p += 2;
    put_u16le(&out[p], (uint16_t)w_est_q15); p += 2;
    out[p++] = st->enc_fault_mask;

    /* 编码器速度数据 (4轮) */
    for (int i = 0; i < ENC_COUNT; i++) {
        int16_t vel = clamp_i16_from_f(g_encoders[i].vel_cps);
        put_u16le(&out[p], (uint16_t)vel); p += 2;
    }

    put_u16le(&out[p], (uint16_t)yaw_est_x100); p += 2;
    put_u16le(&out[p], (uint16_t)raw_ax_mg); p += 2;
    put_u16le(&out[p], (uint16_t)raw_ay_mg); p += 2;
    put_u16le(&out[p], (uint16_t)raw_az_mg); p += 2;
    out[p++] = st->imu_accel_valid;
    out[p++] = st->cmd_semantics;
    put_u16le(&out[p], (uint16_t)raw_left_q15); p += 2;
    put_u16le(&out[p], (uint16_t)raw_right_q15); p += 2;

    *out_len = p;
}

static void append_diag_payload(link_id_t link_id, uint8_t *out, uint16_t *out_len)
{
    link_diag_snapshot_t s;
    uint16_t p = 0;

    LinkDiag_GetSnapshot(link_id, &s);

    put_u32le(&out[p], s.rx_total); p += 4;
    put_u32le(&out[p], s.rx_ok); p += 4;
    put_u32le(&out[p], s.rx_drop); p += 4;
    put_u32le(&out[p], s.cobs_err); p += 4;
    put_u32le(&out[p], s.crc_err); p += 4;
    put_u32le(&out[p], s.len_err); p += 4;
    put_u32le(&out[p], s.sof_err); p += 4;
    put_u32le(&out[p], s.eof_err); p += 4;
    put_u32le(&out[p], s.seq_dup); p += 4;
    put_u32le(&out[p], s.uart_err); p += 4;
    put_u32le(&out[p], s.tx_fail); p += 4;
    put_u16le(&out[p], s.last_err); p += 2;

    *out_len = p;
}

static void append_fault_log_payload(const uint8_t *req_payload,
                                     uint16_t req_len,
                                     uint8_t *out,
                                     uint16_t *out_len)
{
    uint8_t cursor = 0u;
    uint8_t max_records = 4u;
    uint8_t next_cursor = 0u;
    fault_log_record_t recs[8];
    uint16_t count;
    uint16_t p = 0;
    uint16_t i;

    if (req_len >= 1u) cursor = req_payload[0];
    if (req_len >= 2u) max_records = req_payload[1];
    if (max_records == 0u) max_records = 1u;
    if (max_records > 8u) max_records = 8u;

    count = LinkDiag_GetFaultLog(cursor, recs, max_records, &next_cursor);

    out[p++] = next_cursor;
    out[p++] = (uint8_t)count;
    put_u16le(&out[p], (uint16_t)LinkDiag_GetDroppedFaultCount()); p += 2;

    for (i = 0; i < count; ++i) {
        put_u32le(&out[p], recs[i].ts_ms); p += 4;
        out[p++] = recs[i].link_id;
        out[p++] = recs[i].severity;
        put_u16le(&out[p], recs[i].err_code); p += 2;
        out[p++] = recs[i].seq;
        out[p++] = recs[i].msg_type;
        put_u16le(&out[p], recs[i].aux0); p += 2;
        put_u16le(&out[p], recs[i].aux1); p += 2;
    }

    *out_len = p;
}

static void handle_command(link_id_t link_id,
                           uint8_t seq,
                           uint8_t msg_type,
                           const uint8_t *payload,
                           uint16_t payload_len,
                           uint16_t *err_code,
                           uint8_t *detail,
                           uint8_t *resp_payload,
                           uint16_t *resp_len)
{
    uint32_t now_ms = HAL_GetTick();

    *err_code = LINK_ERR_OK;
    *detail = 0u;
    *resp_len = 0u;

    switch (msg_type) {
    case LINK_MSG_CMD_PING:
        RobotControl_NotifyLinkActivityEx(link_id == LINK_ID_PC ? CMD_SRC_PC : CMD_SRC_ESP,
                                          LINK_ACTIVITY_QUERY, now_ms);
        if (payload_len != 0u) {
            *err_code = LINK_ERR_PARSE_PAYLOAD_LEN;
            break;
        }
        resp_payload[0] = 0xAAu;
        *resp_len = 1u;
        break;

    case LINK_MSG_CMD_GET_STATUS:
        RobotControl_NotifyLinkActivityEx(link_id == LINK_ID_PC ? CMD_SRC_PC : CMD_SRC_ESP,
                                          LINK_ACTIVITY_QUERY, now_ms);
        if (payload_len != 0u) {
            *err_code = LINK_ERR_PARSE_PAYLOAD_LEN;
            break;
        }
        append_status_payload(resp_payload, resp_len, NULL, NULL);
        break;

    case LINK_MSG_CMD_SET_DRIVE:
        RobotControl_NotifyLinkActivityEx(link_id == LINK_ID_PC ? CMD_SRC_PC : CMD_SRC_ESP,
                                          LINK_ACTIVITY_CONTROL, now_ms);
        if (payload_len != 4u) {
            *err_code = LINK_ERR_PARSE_PAYLOAD_LEN;
            break;
        }
        if (RobotControl_GetMode() != MODE_CLOSED_LOOP) {
            *err_code = LINK_ERR_CTRL_MODE_REJECT;
            break;
        }
        {
            int16_t v_q15 = (int16_t)get_u16le(&payload[0]);
            int16_t w_q15 = (int16_t)get_u16le(&payload[2]);
            float v = (float)v_q15 / 32767.0f;
            float w = (float)w_q15 / 32767.0f;
            if (link_id == LINK_ID_ESP) RobotControl_SetCmd_ESP(v, w, now_ms);
            else RobotControl_SetCmd_PC(v, w, now_ms);
        }
        break;

    case LINK_MSG_CMD_SET_MODE:
        RobotControl_NotifyLinkActivityEx(link_id == LINK_ID_PC ? CMD_SRC_PC : CMD_SRC_ESP,
                                          LINK_ACTIVITY_QUERY, now_ms);
        if (payload_len != 1u) {
            *err_code = LINK_ERR_PARSE_PAYLOAD_LEN;
            break;
        }
        if (payload[0] > 2u) {
            *err_code = LINK_ERR_PARSE_PARAM_RANGE;
            break;
        }
        RobotControl_SetMode((ControlMode_t)payload[0]);
        break;

    case LINK_MSG_CMD_SET_IMU:
        RobotControl_NotifyLinkActivityEx(link_id == LINK_ID_PC ? CMD_SRC_PC : CMD_SRC_ESP,
                                          LINK_ACTIVITY_QUERY, now_ms);
        if (payload_len != 1u) {
            *err_code = LINK_ERR_PARSE_PAYLOAD_LEN;
            break;
        }
        RobotControl_SetIMUEnabled(payload[0] ? 1u : 0u);
        break;

    case LINK_MSG_CMD_SET_RAW:
        RobotControl_NotifyLinkActivityEx(link_id == LINK_ID_PC ? CMD_SRC_PC : CMD_SRC_ESP,
                                          LINK_ACTIVITY_CONTROL, now_ms);
        if (payload_len != 4u) {
            *err_code = LINK_ERR_PARSE_PAYLOAD_LEN;
            break;
        }
        if (RobotControl_GetMode() != MODE_OPEN_LOOP) {
            *err_code = LINK_ERR_CTRL_MODE_REJECT;
            break;
        }
        {
            int16_t left_q15 = (int16_t)get_u16le(&payload[0]);
            int16_t right_q15 = (int16_t)get_u16le(&payload[2]);
            float left = (float)left_q15 / 32767.0f;
            float right = (float)right_q15 / 32767.0f;
            RobotControl_SetOpenLoopCmd(left,
                                        right,
                                        link_id == LINK_ID_PC ? CMD_SRC_PC : CMD_SRC_ESP,
                                        now_ms);
        }
        break;

    case LINK_MSG_CMD_GET_DIAG:
        RobotControl_NotifyLinkActivityEx(link_id == LINK_ID_PC ? CMD_SRC_PC : CMD_SRC_ESP,
                                          LINK_ACTIVITY_QUERY, now_ms);
        if (payload_len != 0u) {
            *err_code = LINK_ERR_PARSE_PAYLOAD_LEN;
            break;
        }
        append_diag_payload(link_id, resp_payload, resp_len);
        break;

    case LINK_MSG_CMD_CLEAR_DIAG:
        RobotControl_NotifyLinkActivityEx(link_id == LINK_ID_PC ? CMD_SRC_PC : CMD_SRC_ESP,
                                          LINK_ACTIVITY_QUERY, now_ms);
        if (payload_len != 0u) {
            *err_code = LINK_ERR_PARSE_PAYLOAD_LEN;
            break;
        }
        LinkDiag_ClearSnapshot(link_id);
        break;

    case LINK_MSG_CMD_GET_FAULT_LOG:
        RobotControl_NotifyLinkActivityEx(link_id == LINK_ID_PC ? CMD_SRC_PC : CMD_SRC_ESP,
                                          LINK_ACTIVITY_QUERY, now_ms);
        if (payload_len > 2u) {
            *err_code = LINK_ERR_PARSE_PAYLOAD_LEN;
            break;
        }
        append_fault_log_payload(payload, payload_len, resp_payload, resp_len);
        break;

    case LINK_MSG_CMD_CLEAR_FAULT_LOG:
        RobotControl_NotifyLinkActivityEx(link_id == LINK_ID_PC ? CMD_SRC_PC : CMD_SRC_ESP,
                                          LINK_ACTIVITY_QUERY, now_ms);
        if (payload_len != 0u) {
            *err_code = LINK_ERR_PARSE_PAYLOAD_LEN;
            break;
        }
        LinkDiag_ClearFaultLog();
        break;

    default:
        *err_code = LINK_ERR_PARSE_UNKNOWN_CMD;
        break;
    }

    if (*err_code != LINK_ERR_OK) {
        LinkDiag_Count(link_id, *err_code);
        LinkDiag_PushFault(link_id, FAULT_SEV_ERROR, *err_code, seq, msg_type, payload_len, 0u);
    }
}

static void send_ack_or_nack(link_id_t link_id,
                             uint8_t seq,
                             uint8_t is_ack,
                             uint16_t err_code,
                             uint8_t detail,
                             const uint8_t *extra,
                             uint16_t extra_len)
{
    uint8_t payload[LINK_RESP_MAX];
    uint16_t p = 0;

    payload[p++] = seq;
    put_u16le(&payload[p], err_code); p += 2;
    payload[p++] = detail;

    if (is_ack && extra && extra_len > 0u) {
        if ((uint16_t)(p + extra_len) > sizeof(payload)) {
            extra_len = (uint16_t)(sizeof(payload) - p);
        }
        memcpy(&payload[p], extra, extra_len);
        p = (uint16_t)(p + extra_len);
    }

    (void)link_send_frame(link_id,
                          is_ack ? LINK_MSG_ACK : LINK_MSG_NACK,
                          is_ack ? LINK_FLAG_IS_ACK : LINK_FLAG_IS_NACK,
                          seq,
                          payload,
                          p,
                          1u);
}

void LinkProto_Init(link_id_t link_id, UART_HandleTypeDef *huart)
{
    if (!valid_link(link_id)) return;

    if (!s_diag_inited) {
        LinkDiag_Init();
        s_diag_inited = 1u;
    }

    memset(&s_ctx[link_id], 0, sizeof(s_ctx[link_id]));
    s_ctx[link_id].huart = huart;
    LinkDiag_ResetLink(link_id);
}

void LinkProto_RxByte(link_id_t link_id, uint8_t byte)
{
    link_ctx_t *ctx;

    if (!valid_link(link_id)) return;
    ctx = &s_ctx[link_id];

    if (byte == 0x00u) {
        if (ctx->rx_accum_len > 0u) {
            if (ctx->rx_packet_ready) {
                LinkDiag_IncRxDrop(link_id);
                LinkDiag_Count(link_id, LINK_ERR_RES_RX_OVERFLOW);
            } else {
                memcpy(ctx->rx_packet, ctx->rx_accum, ctx->rx_accum_len);
                ctx->rx_packet_len = ctx->rx_accum_len;
                ctx->rx_packet_ready = 1u;
            }
        }
        ctx->rx_accum_len = 0u;
        return;
    }

    if (ctx->rx_accum_len >= sizeof(ctx->rx_accum)) {
        ctx->rx_accum_len = 0u;
        LinkDiag_IncRxDrop(link_id);
        LinkDiag_Count(link_id, LINK_ERR_RES_RX_OVERFLOW);
        LinkDiag_PushFault(link_id, FAULT_SEV_WARN, LINK_ERR_RES_RX_OVERFLOW, 0u, 0u, 0u, 0u);
        return;
    }

    ctx->rx_accum[ctx->rx_accum_len++] = byte;
}

void LinkProto_UartError(link_id_t link_id)
{
    link_ctx_t *ctx;

    if (!valid_link(link_id)) return;
    ctx = &s_ctx[link_id];

    __disable_irq();
    ctx->tx_busy = 0u;
    ctx->tx_head = 0u;
    ctx->tx_tail = 0u;
    ctx->tx_count = 0u;
    __enable_irq();

    LinkDiag_IncUartErr(link_id);
    LinkDiag_Count(link_id, LINK_ERR_UART_IO);
    LinkDiag_PushFault(link_id, FAULT_SEV_ERROR, LINK_ERR_UART_IO, 0u, 0u, 0u, 0u);
}

HAL_StatusTypeDef LinkProto_SendRaw(link_id_t link_id, const uint8_t *data, uint16_t len)
{
    link_ctx_t *ctx;
    HAL_StatusTypeDef st;

    if (!valid_link(link_id) || !data || len == 0u) return HAL_ERROR;
    ctx = &s_ctx[link_id];
    if (!ctx->huart) return HAL_ERROR;

    st = link_send_encoded(ctx, data, len);
    if (st == HAL_OK) {
        link_kick_tx(ctx);
    }
    return st;
}

void LinkProto_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    int link_id;

    if (!huart) return;

    for (link_id = 0; link_id < LINK_ID_MAX; ++link_id) {
        link_ctx_t *ctx = &s_ctx[link_id];
        if (ctx->huart != huart) continue;

        __disable_irq();
        if (ctx->tx_count > 0u) {
            ctx->tx_head = (uint8_t)((ctx->tx_head + 1u) % LINK_TX_QUEUE_DEPTH);
            ctx->tx_count--;
        }
        ctx->tx_busy = 0u;
        __enable_irq();
        link_kick_tx(ctx);
    }
}

void LinkProto_Poll(link_id_t link_id,
                    const BatteryStatus_t *batt,
                    const MPU6050_Data_t *imu)
{
    link_ctx_t *ctx;
    uint8_t pkt[LINK_RX_ACCUM_MAX];
    uint16_t pkt_len = 0;
    uint8_t dec[LINK_RESP_MAX];
    uint16_t dec_len;
    uint8_t ver;
    uint8_t msg_type;
    uint8_t flags;
    uint8_t seq;
    uint16_t payload_len;
    const uint8_t *payload;
    uint16_t recv_crc;
    uint16_t calc_crc;
    uint16_t err_code = LINK_ERR_OK;
    uint8_t detail = 0u;
    uint8_t resp_payload[LINK_RESP_MAX];
    uint16_t resp_len = 0u;

    if (!valid_link(link_id)) return;

    ctx = &s_ctx[link_id];
    if (!ctx->huart) return;
    link_kick_tx(ctx);
    if (!ctx->rx_packet_ready) return;

    __disable_irq();
    if (!ctx->rx_packet_ready) {
        __enable_irq();
        return;
    }
    pkt_len = ctx->rx_packet_len;
    if (pkt_len > sizeof(pkt)) pkt_len = sizeof(pkt);
    memcpy(pkt, ctx->rx_packet, pkt_len);
    ctx->rx_packet_ready = 0u;
    ctx->rx_packet_len = 0u;
    __enable_irq();

    LinkDiag_IncRxTotal(link_id);

    dec_len = cobs_decode(pkt, pkt_len, dec, sizeof(dec));
    if (dec_len == 0u) {
        LinkDiag_IncCobsErr(link_id);
        LinkDiag_Count(link_id, LINK_ERR_PROTO_COBS);
        LinkDiag_PushFault(link_id, FAULT_SEV_WARN, LINK_ERR_PROTO_COBS, 0u, 0u, pkt_len, 0u);
        return;
    }

    if (dec_len < LINK_FRAME_MIN) {
        LinkDiag_IncLenErr(link_id);
        LinkDiag_Count(link_id, LINK_ERR_PROTO_LEN);
        LinkDiag_PushFault(link_id, FAULT_SEV_WARN, LINK_ERR_PROTO_LEN, 0u, 0u, dec_len, 0u);
        return;
    }

    if (dec[0] != LINK_SOF) {
        LinkDiag_IncSofErr(link_id);
        LinkDiag_Count(link_id, LINK_ERR_PROTO_SOF);
        LinkDiag_PushFault(link_id, FAULT_SEV_WARN, LINK_ERR_PROTO_SOF, 0u, 0u, dec[0], 0u);
        return;
    }

    if (dec[dec_len - 1u] != LINK_EOF) {
        LinkDiag_IncEofErr(link_id);
        LinkDiag_Count(link_id, LINK_ERR_PROTO_EOF);
        LinkDiag_PushFault(link_id, FAULT_SEV_WARN, LINK_ERR_PROTO_EOF, 0u, 0u, dec[dec_len - 1u], 0u);
        return;
    }

    ver = dec[1];
    msg_type = dec[2];
    flags = dec[3];
    seq = dec[4];
    payload_len = get_u16le(&dec[5]);

    if ((uint16_t)(payload_len + LINK_FRAME_MIN) != dec_len) {
        LinkDiag_IncLenErr(link_id);
        LinkDiag_Count(link_id, LINK_ERR_PROTO_LEN);
        LinkDiag_PushFault(link_id, FAULT_SEV_WARN, LINK_ERR_PROTO_LEN, seq, msg_type, payload_len, dec_len);
        return;
    }

    recv_crc = get_u16le(&dec[7 + payload_len]);
    calc_crc = crc16_ccitt_false(&dec[1], (uint16_t)(6u + payload_len));
    if (recv_crc != calc_crc) {
        LinkDiag_IncCrcErr(link_id);
        LinkDiag_Count(link_id, LINK_ERR_PROTO_CRC);
        LinkDiag_PushFault(link_id, FAULT_SEV_WARN, LINK_ERR_PROTO_CRC, seq, msg_type, recv_crc, calc_crc);
        return;
    }

    if (ver != LINK_PROTO_VER) {
        err_code = LINK_ERR_PARSE_PARAM_RANGE;
    }

    payload = &dec[7];

    if (msg_type == LINK_MSG_ACK || msg_type == LINK_MSG_NACK) {
        LinkDiag_IncRxOk(link_id);
        return;
    }

    if (ctx->last_req_valid &&
        msg_type != LINK_MSG_CMD_SET_DRIVE &&
        msg_type != LINK_MSG_CMD_SET_RAW &&
        ctx->last_req_seq == seq &&
        ctx->last_req_type == msg_type) {
        LinkDiag_IncSeqDup(link_id);
        LinkDiag_Count(link_id, LINK_ERR_PROTO_SEQ);
        if (ctx->last_resp_valid && ctx->last_resp_seq == seq) {
            (void)link_send_encoded(ctx, ctx->last_resp_data, ctx->last_resp_len);
        }
        return;
    }

    ctx->last_req_valid = 1u;
    ctx->last_req_seq = seq;
    ctx->last_req_type = msg_type;

    if (err_code == LINK_ERR_OK) {
        handle_command(link_id, seq, msg_type, payload, payload_len,
                       &err_code, &detail, resp_payload, &resp_len);
    }

    if (flags & LINK_FLAG_ACK_REQ) {
        if (err_code == LINK_ERR_OK) {
            if (msg_type == LINK_MSG_CMD_GET_STATUS) {
                append_status_payload(resp_payload, &resp_len, batt, imu);
            }
            send_ack_or_nack(link_id, seq, 1u, LINK_ERR_OK, detail, resp_payload, resp_len);
        } else {
            send_ack_or_nack(link_id, seq, 0u, err_code, detail, NULL, 0u);
        }
    }

    LinkDiag_IncRxOk(link_id);
}
