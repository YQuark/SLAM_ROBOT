#include "pc_link.h"
#include "link_proto.h"
#include "robot_control.h"
#include "encoder.h"
#include "motor.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PC_ASCII_LINE_MAX 96u
#define PC_STREAM_HZ_DEFAULT 20u
#define PC_STREAM_HZ_MIN 1u
#define PC_STREAM_HZ_MAX 50u

static UART_HandleTypeDef *s_huart = NULL;
static uint8_t s_rx_ch = 0u;
static char s_ascii_accum[PC_ASCII_LINE_MAX];
static uint16_t s_ascii_accum_len = 0u;
static char s_ascii_line[PC_ASCII_LINE_MAX];
static volatile uint8_t s_ascii_line_ready = 0u;
static uint8_t s_stream_enable = 0u;
static uint16_t s_stream_hz = PC_STREAM_HZ_DEFAULT;
static uint32_t s_stream_last_ms = 0u;
static uint8_t s_motor_override = 0u;
static float s_motor_left = 0.0f;
static float s_motor_right = 0.0f;

static int32_t scaled_i32(float value, float scale)
{
    float scaled = value * scale;
    if (scaled > 2147483647.0f) return 2147483647;
    if (scaled < -2147483648.0f) return (-2147483647 - 1);
    return (int32_t)scaled;
}

static void pc_uart_send(const char *text)
{
    if (!s_huart || !text) return;
    (void)HAL_UART_Transmit(s_huart, (uint8_t *)text, (uint16_t)strlen(text), 20);
}

static void pc_uart_sendf(const char *fmt, ...)
{
    char out[160];
    va_list args;
    int n;

    if (!fmt) return;
    va_start(args, fmt);
    n = vsnprintf(out, sizeof(out), fmt, args);
    va_end(args);
    if (n <= 0) return;
    if ((size_t)n >= sizeof(out)) {
        n = (int)(sizeof(out) - 1u);
    }
    if (!s_huart) return;
    (void)HAL_UART_Transmit(s_huart, (uint8_t *)out, (uint16_t)n, 20);
}

static uint8_t str_ieq(const char *a, const char *b)
{
    if (!a || !b) return 0u;
    while (*a && *b) {
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b)) {
            return 0u;
        }
        ++a;
        ++b;
    }
    return (*a == '\0' && *b == '\0') ? 1u : 0u;
}

static char *trim_left(char *s)
{
    if (!s) return s;
    while (*s && isspace((unsigned char)*s)) ++s;
    return s;
}

static void trim_right(char *s)
{
    size_t len;
    if (!s) return;
    len = strlen(s);
    while (len > 0u && isspace((unsigned char)s[len - 1u])) {
        s[len - 1u] = '\0';
        --len;
    }
}

static uint8_t parse_mode_token(const char *tok, ControlMode_t *mode_out)
{
    long v;
    char *endptr = NULL;

    if (!tok || !mode_out) return 0u;

    v = strtol(tok, &endptr, 10);
    if (endptr && *endptr == '\0') {
        if (v >= MODE_IDLE && v <= MODE_CLOSED_LOOP) {
            *mode_out = (ControlMode_t)v;
            return 1u;
        }
        return 0u;
    }

    if (str_ieq(tok, "IDLE")) {
        *mode_out = MODE_IDLE;
        return 1u;
    }
    if (str_ieq(tok, "OPEN") || str_ieq(tok, "OPEN_LOOP") || str_ieq(tok, "OL")) {
        *mode_out = MODE_OPEN_LOOP;
        return 1u;
    }
    if (str_ieq(tok, "CLOSED") || str_ieq(tok, "CLOSED_LOOP") || str_ieq(tok, "CL")) {
        *mode_out = MODE_CLOSED_LOOP;
        return 1u;
    }
    return 0u;
}

static uint8_t parse_bool_token(const char *tok, uint8_t *out)
{
    if (!tok || !out) return 0u;
    if (str_ieq(tok, "1") || str_ieq(tok, "ON") || str_ieq(tok, "TRUE")) {
        *out = 1u;
        return 1u;
    }
    if (str_ieq(tok, "0") || str_ieq(tok, "OFF") || str_ieq(tok, "FALSE")) {
        *out = 0u;
        return 1u;
    }
    return 0u;
}

static uint8_t parse_float_token(const char *tok, float *out)
{
    char *endptr = NULL;
    float v;
    if (!tok || !out) return 0u;
    v = strtof(tok, &endptr);
    if (!endptr || *endptr != '\0') return 0u;
    *out = v;
    return 1u;
}

static uint8_t parse_u16_token(const char *tok, uint16_t *out)
{
    long v;
    char *endptr = NULL;
    if (!tok || !out) return 0u;
    v = strtol(tok, &endptr, 10);
    if (!endptr || *endptr != '\0') return 0u;
    if (v < 0 || v > 65535) return 0u;
    *out = (uint16_t)v;
    return 1u;
}

static uint32_t stream_interval_ms(void)
{
    if (s_stream_hz == 0u) return 1000u;
    return (uint32_t)(1000u / s_stream_hz);
}

static void PC_Link_StreamOnce(const BatteryStatus_t *batt,
                               const MPU6050_Data_t *imu)
{
    const RobotControlState_t *st = RobotControl_GetState();
    int32_t vbatt_mv = batt ? scaled_i32(batt->voltage, 1000.0f) : 0;
    int32_t pct_x10 = batt ? scaled_i32(batt->percent, 10.0f) : 0;
    int32_t ax_mg = imu ? scaled_i32(imu->ax_g, 1000.0f) : 0;
    int32_t ay_mg = imu ? scaled_i32(imu->ay_g, 1000.0f) : 0;
    int32_t az_mg = imu ? scaled_i32(imu->az_g, 1000.0f) : 0;
    int32_t gx_cdps = imu ? scaled_i32(imu->gx_dps, 100.0f) : 0;
    int32_t gy_cdps = imu ? scaled_i32(imu->gy_dps, 100.0f) : 0;
    int32_t gz_cdps = imu ? scaled_i32(imu->gz_dps, 100.0f) : 0;
    int32_t vel_l1 = scaled_i32(g_encoders[ENC_L1].vel_cps, 1.0f);
    int32_t vel_l2 = scaled_i32(g_encoders[ENC_L2].vel_cps, 1.0f);
    int32_t vel_r1 = scaled_i32(g_encoders[ENC_R1].vel_cps, 1.0f);
    int32_t vel_r2 = scaled_i32(g_encoders[ENC_R2].vel_cps, 1.0f);

    pc_uart_sendf(
        "TEL,%lu,%u,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
        (unsigned long)HAL_GetTick(),
        (unsigned int)RobotControl_GetMode(),
        (unsigned int)st->src,
        (long)ax_mg, (long)ay_mg, (long)az_mg,
        (long)gx_cdps, (long)gy_cdps, (long)gz_cdps,
        (long)g_encoders[ENC_L1].pos,
        (long)g_encoders[ENC_L2].pos,
        (long)g_encoders[ENC_R1].pos,
        (long)g_encoders[ENC_R2].pos,
        (long)vel_l1, (long)vel_l2, (long)vel_r1, (long)vel_r2,
        (long)vbatt_mv, (long)pct_x10);
}

static void PC_Link_StreamTick(const BatteryStatus_t *batt,
                               const MPU6050_Data_t *imu)
{
    uint32_t now;
    uint32_t interval;

    if (!s_stream_enable) return;
    now = HAL_GetTick();
    interval = stream_interval_ms();
    if ((now - s_stream_last_ms) < interval) return;
    s_stream_last_ms = now;
    PC_Link_StreamOnce(batt, imu);
}

static uint8_t is_ascii_cmd_byte(uint8_t ch)
{
    return (ch >= 0x20u && ch <= 0x7Eu) || (ch == '\t');
}

static uint8_t is_line_end_byte(uint8_t ch)
{
    return (ch == '\r' || ch == '\n') ? 1u : 0u;
}

static void ascii_accum_reset(void)
{
    s_ascii_accum_len = 0u;
}

static void ascii_accum_push(uint8_t ch)
{
    if (s_ascii_accum_len >= (PC_ASCII_LINE_MAX - 1u)) {
        ascii_accum_reset();
        return;
    }
    s_ascii_accum[s_ascii_accum_len++] = (char)ch;
}

static void ascii_finalize_line_from_accum(void)
{
    if (s_ascii_accum_len == 0u) return;
    if (!isalpha((unsigned char)s_ascii_accum[0])) {
        ascii_accum_reset();
        return;
    }
    if (!s_ascii_line_ready) {
        memcpy(s_ascii_line, s_ascii_accum, s_ascii_accum_len);
        s_ascii_line[s_ascii_accum_len] = '\0';
        s_ascii_line_ready = 1u;
    }
    ascii_accum_reset();
}

static void PC_Link_AsciiTapByte(uint8_t ch)
{
    if (is_line_end_byte(ch)) {
        ascii_finalize_line_from_accum();
        return;
    }
    if (is_ascii_cmd_byte(ch)) {
        ascii_accum_push(ch);
        return;
    }
    ascii_accum_reset();
}

static void PC_Link_HandleAsciiLine(char *line,
                                    const BatteryStatus_t *batt,
                                    const MPU6050_Data_t *imu)
{
    char *cmd;
    char *arg1;
    char *arg2;
    const RobotControlState_t *st;
    uint8_t imu_en = 0u;
    ControlMode_t mode;
    float v;
    float w;
    uint16_t hz = 0u;
    uint16_t ccr[4] = {0u, 0u, 0u, 0u};
    uint8_t dir[4] = {0u, 0u, 0u, 0u};

    if (!line) return;
    line = trim_left(line);
    trim_right(line);
    if (line[0] == '\0') return;

    cmd = strtok(line, " \t");
    arg1 = strtok(NULL, " \t");
    arg2 = strtok(NULL, " \t");
    if (!cmd) return;

    if (str_ieq(cmd, "PING")) {
        pc_uart_send("OK PONG\r\n");
        return;
    }

    if (str_ieq(cmd, "HELP")) {
        pc_uart_send("OK CMDS: PING, HELP, STATUS, DEBUG, MODE <0|1|2>, DRIVE <v> <w>, CTRL <v> <w>, MOTOR <l> <r>, STOP, IMU <0|1>, STREAM <ON|OFF> [hz], RATE <hz>\r\n");
        return;
    }

    if (str_ieq(cmd, "STATUS")) {
        st = RobotControl_GetState();
        pc_uart_sendf("OK STATUS mode=%u src=%u v_x1000=%ld w_x1000=%ld vbat_mv=%ld imu_en=%u imu_ok=%u gz_cdps=%ld motor_ovr=%u\r\n",
                      (unsigned int)RobotControl_GetMode(),
                      (unsigned int)st->src,
                      (long)scaled_i32(st->v_cmd, 1000.0f),
                      (long)scaled_i32(st->w_cmd, 1000.0f),
                      (long)(batt ? scaled_i32(batt->voltage, 1000.0f) : 0),
                      (unsigned int)st->imu_enabled,
                      (unsigned int)st->imu_valid,
                      (long)(imu ? scaled_i32(imu->gz_dps, 100.0f) : 0),
                      (unsigned int)s_motor_override);
        return;
    }

    if (str_ieq(cmd, "DEBUG")) {
        st = RobotControl_GetState();
        motor_get_debug(ccr, dir);
        pc_uart_sendf(
            "OK DEBUG mode=%u src=%u ref=%ld,%ld,%ld,%ld meas=%ld,%ld,%ld,%ld u_x1000=%ld,%ld,%ld,%ld ccr=%u,%u,%u,%u dir=%u,%u,%u,%u fault=0x%02X\r\n",
            (unsigned int)RobotControl_GetMode(),
            (unsigned int)st->src,
            (long)scaled_i32(st->ref_cps[0], 1.0f),
            (long)scaled_i32(st->ref_cps[1], 1.0f),
            (long)scaled_i32(st->ref_cps[2], 1.0f),
            (long)scaled_i32(st->ref_cps[3], 1.0f),
            (long)scaled_i32(st->meas_cps[0], 1.0f),
            (long)scaled_i32(st->meas_cps[1], 1.0f),
            (long)scaled_i32(st->meas_cps[2], 1.0f),
            (long)scaled_i32(st->meas_cps[3], 1.0f),
            (long)scaled_i32(st->u_out[0], 1000.0f),
            (long)scaled_i32(st->u_out[1], 1000.0f),
            (long)scaled_i32(st->u_out[2], 1000.0f),
            (long)scaled_i32(st->u_out[3], 1000.0f),
            (unsigned int)ccr[0], (unsigned int)ccr[1], (unsigned int)ccr[2], (unsigned int)ccr[3],
            (unsigned int)dir[0], (unsigned int)dir[1], (unsigned int)dir[2], (unsigned int)dir[3],
            (unsigned int)st->enc_fault_mask);
        return;
    }

    if (str_ieq(cmd, "MODE")) {
        if (!arg1) {
            pc_uart_sendf("OK MODE %u\r\n", (unsigned int)RobotControl_GetMode());
            return;
        }
        if (!parse_mode_token(arg1, &mode)) {
            pc_uart_send("ERR MODE expect 0|1|2 or IDLE|OPEN|CLOSED\r\n");
            return;
        }
        RobotControl_SetMode(mode);
        pc_uart_sendf("OK MODE %u\r\n", (unsigned int)mode);
        return;
    }

    if (str_ieq(cmd, "IMU")) {
        st = RobotControl_GetState();
        if (!arg1) {
            pc_uart_sendf("OK IMU %u\r\n", (unsigned int)st->imu_enabled);
            return;
        }
        if (!parse_bool_token(arg1, &imu_en)) {
            pc_uart_send("ERR IMU expect 0|1 or OFF|ON\r\n");
            return;
        }
        RobotControl_SetIMUEnabled(imu_en);
        pc_uart_sendf("OK IMU %u\r\n", (unsigned int)imu_en);
        return;
    }

    if (str_ieq(cmd, "STOP")) {
        s_motor_override = 0u;
        s_motor_left = 0.0f;
        s_motor_right = 0.0f;
        motor_coast_all();
        RobotControl_SetCmd_PC(0.0f, 0.0f, HAL_GetTick());
        pc_uart_send("OK STOP\r\n");
        return;
    }

    if (str_ieq(cmd, "DRIVE") || str_ieq(cmd, "CTRL")) {
        if (!arg1 || !arg2) {
            pc_uart_send("ERR DRIVE expect: DRIVE <v> <w>\r\n");
            return;
        }
        if (!parse_float_token(arg1, &v) || !parse_float_token(arg2, &w)) {
            pc_uart_send("ERR DRIVE bad number\r\n");
            return;
        }
        if (v > 1.0f) v = 1.0f;
        if (v < -1.0f) v = -1.0f;
        if (w > 1.0f) w = 1.0f;
        if (w < -1.0f) w = -1.0f;
        s_motor_override = 0u;
        s_motor_left = 0.0f;
        s_motor_right = 0.0f;
        RobotControl_SetCmd_PC(v, w, HAL_GetTick());
        pc_uart_sendf("OK DRIVE v_x1000=%ld w_x1000=%ld\r\n",
                      (long)scaled_i32(v, 1000.0f),
                      (long)scaled_i32(w, 1000.0f));
        return;
    }

    if (str_ieq(cmd, "MOTOR")) {
        if (!arg1 || !arg2) {
            pc_uart_send("ERR MOTOR expect: MOTOR <left> <right>\r\n");
            return;
        }
        if (!parse_float_token(arg1, &v) || !parse_float_token(arg2, &w)) {
            pc_uart_send("ERR MOTOR bad number\r\n");
            return;
        }
        if (v > 1.0f) v = 1.0f;
        if (v < -1.0f) v = -1.0f;
        if (w > 1.0f) w = 1.0f;
        if (w < -1.0f) w = -1.0f;
        s_motor_override = 1u;
        s_motor_left = v;
        s_motor_right = w;
        motor_set(MOTOR_L1, v);
        motor_set(MOTOR_L2, v);
        motor_set(MOTOR_R1, w);
        motor_set(MOTOR_R2, w);
        Encoder_SetMotorOutput(ENC_L1, v);
        Encoder_SetMotorOutput(ENC_L2, v);
        Encoder_SetMotorOutput(ENC_R1, w);
        Encoder_SetMotorOutput(ENC_R2, w);
        pc_uart_sendf("OK MOTOR l_x1000=%ld r_x1000=%ld\r\n",
                      (long)scaled_i32(v, 1000.0f),
                      (long)scaled_i32(w, 1000.0f));
        return;
    }

    if (str_ieq(cmd, "RATE")) {
        if (!arg1 || !parse_u16_token(arg1, &hz)) {
            pc_uart_sendf("ERR RATE expect %u..%u\r\n", (unsigned int)PC_STREAM_HZ_MIN, (unsigned int)PC_STREAM_HZ_MAX);
            return;
        }
        if (hz < PC_STREAM_HZ_MIN || hz > PC_STREAM_HZ_MAX) {
            pc_uart_sendf("ERR RATE range %u..%u\r\n", (unsigned int)PC_STREAM_HZ_MIN, (unsigned int)PC_STREAM_HZ_MAX);
            return;
        }
        s_stream_hz = hz;
        pc_uart_sendf("OK RATE %uHz\r\n", (unsigned int)s_stream_hz);
        return;
    }

    if (str_ieq(cmd, "STREAM")) {
        if (!arg1) {
            pc_uart_sendf("OK STREAM %s %uHz\r\n",
                          s_stream_enable ? "ON" : "OFF",
                          (unsigned int)s_stream_hz);
            return;
        }
        if (str_ieq(arg1, "OFF") || str_ieq(arg1, "0")) {
            s_stream_enable = 0u;
            pc_uart_sendf("OK STREAM OFF %uHz\r\n", (unsigned int)s_stream_hz);
            return;
        }
        if (str_ieq(arg1, "ON") || str_ieq(arg1, "1")) {
            if (arg2) {
                if (!parse_u16_token(arg2, &hz) ||
                    hz < PC_STREAM_HZ_MIN || hz > PC_STREAM_HZ_MAX) {
                    pc_uart_sendf("ERR STREAM hz range %u..%u\r\n",
                                  (unsigned int)PC_STREAM_HZ_MIN,
                                  (unsigned int)PC_STREAM_HZ_MAX);
                    return;
                }
                s_stream_hz = hz;
            }
            s_stream_enable = 1u;
            s_stream_last_ms = 0u;
            pc_uart_sendf("OK STREAM ON %uHz\r\n", (unsigned int)s_stream_hz);
            return;
        }
        pc_uart_send("ERR STREAM expect ON|OFF [hz]\r\n");
        return;
    }

    pc_uart_send("ERR UNKNOWN_CMD\r\n");
}

void PC_Link_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    ascii_accum_reset();
    s_ascii_line_ready = 0u;
    s_ascii_line[0] = '\0';
    s_stream_enable = 0u;
    s_stream_hz = PC_STREAM_HZ_DEFAULT;
    s_stream_last_ms = 0u;
    LinkProto_Init(LINK_ID_PC, huart);
    if (s_huart) {
        HAL_UART_Receive_IT(s_huart, &s_rx_ch, 1);
    }
}

void PC_Link_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t ch;

    if (!s_huart || huart != s_huart) return;
    ch = s_rx_ch;

    /* Always feed binary parser first to preserve COBS/framed protocol compatibility. */
    LinkProto_RxByte(LINK_ID_PC, ch);
    /* ASCII command parsing is best-effort and must never block binary traffic. */
    PC_Link_AsciiTapByte(ch);

    HAL_UART_Receive_IT(s_huart, &s_rx_ch, 1);
}

void PC_Link_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (!s_huart || huart != s_huart) return;

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    ascii_accum_reset();
    LinkProto_UartError(LINK_ID_PC);
    HAL_UART_Receive_IT(s_huart, &s_rx_ch, 1);
}

void PC_Link_Poll(const BatteryStatus_t *batt,
                  const MPU6050_Data_t *imu)
{
    char line[PC_ASCII_LINE_MAX];
    uint8_t has_line = 0u;

    __disable_irq();
    if (s_ascii_line_ready) {
        memcpy(line, s_ascii_line, sizeof(line));
        s_ascii_line_ready = 0u;
        has_line = 1u;
    }
    __enable_irq();

    if (has_line) {
        line[PC_ASCII_LINE_MAX - 1u] = '\0';
        PC_Link_HandleAsciiLine(line, batt, imu);
    }

    if (s_motor_override) {
        motor_set(MOTOR_L1, s_motor_left);
        motor_set(MOTOR_L2, s_motor_left);
        motor_set(MOTOR_R1, s_motor_right);
        motor_set(MOTOR_R2, s_motor_right);
        Encoder_SetMotorOutput(ENC_L1, s_motor_left);
        Encoder_SetMotorOutput(ENC_L2, s_motor_left);
        Encoder_SetMotorOutput(ENC_R1, s_motor_right);
        Encoder_SetMotorOutput(ENC_R2, s_motor_right);
    }

    PC_Link_StreamTick(batt, imu);
    LinkProto_Poll(LINK_ID_PC, batt, imu);
}
