// Created by yushu on 2025/12/8.
#include "encoder.h"
#include <math.h>
#include "robot_config.h"
#include <stdio.h>

/* 这些句柄在 tim.c 里定义，这里只做 extern 声明 */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

Encoder_t g_encoders[ENC_COUNT] =
{
    { &htim2, 0, 0, 0.0f, ENC_L1_POLARITY, ENC_STATUS_OK, 0, 0, 0, 0, 0.0f, 0.0f },
    { &htim3, 0, 0, 0.0f, ENC_L2_POLARITY, ENC_STATUS_OK, 0, 0, 0, 0, 0.0f, 0.0f },
    { &htim4, 0, 0, 0.0f, ENC_R1_POLARITY, ENC_STATUS_OK, 0, 0, 0, 0, 0.0f, 0.0f },
    { &htim5, 0, 0, 0.0f, ENC_R2_POLARITY, ENC_STATUS_OK, 0, 0, 0, 0, 0.0f, 0.0f },
};

static float s_vel_bias_cps[ENC_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};

/* ==================== 内部辅助函数 ==================== */

static inline int16_t diff16(uint16_t now16, uint16_t last16)
{
    return (int16_t)(now16 - last16);
}

static inline uint16_t read_cnt16(TIM_HandleTypeDef *htim)
{
    uint32_t c = __HAL_TIM_GET_COUNTER(htim);
    return (uint16_t)(c & 0xFFFFu);
}

/* ==================== 初始化 ==================== */

void Encoder_Init(void)
{
    for (int i = 0; i < ENC_COUNT; ++i) {
        Encoder_t *e = &g_encoders[i];
        e->last_cnt = 0;
        e->pos      = 0;
        e->vel_cps  = 0.0f;

        /* 断线检测初始化 */
        e->status = ENC_STATUS_OK;
        e->fault_ticks = 0;
        e->zero_vel_ticks = 0;
        e->total_glitches = 0;
        e->total_disconnects = 0;
        e->last_valid_vel = 0.0f;
        e->motor_output = 0.0f;

        __HAL_TIM_SET_COUNTER(e->htim, 0);
        HAL_StatusTypeDef status = HAL_TIM_Encoder_Start(e->htim, TIM_CHANNEL_ALL);

        if (status != HAL_OK) {
            extern UART_HandleTypeDef huart1;
            char dbg[64];
            int n = snprintf(dbg, sizeof(dbg),
                           "Encoder %d START FAIL, status=%d\r\n", i, (int)status);
            HAL_UART_Transmit(&huart1, (uint8_t*)dbg, n, 100);
        }

        e->htim->Instance->CR1 &= ~TIM_CR1_UDIS;
        e->last_cnt = read_cnt16(e->htim);
        s_vel_bias_cps[i] = 0.0f;
    }
}

/* ==================== 静止校准 ==================== */

uint8_t Encoder_CalibrateStatic(uint32_t duration_ms, uint32_t sample_interval_ms)
{
    if (duration_ms < 50u) duration_ms = 50u;
    if (sample_interval_ms == 0u) sample_interval_ms = 5u;
    if (sample_interval_ms > duration_ms) sample_interval_ms = duration_ms;

    uint32_t start_ms = HAL_GetTick();
    uint32_t last_ms = start_ms;
    int32_t sum_counts[ENC_COUNT] = {0, 0, 0, 0};
    uint8_t all_ok = 1u;

    for (int i = 0; i < ENC_COUNT; ++i) {
        g_encoders[i].last_cnt = read_cnt16(g_encoders[i].htim);
        s_vel_bias_cps[i] = 0.0f;
    }

    while ((HAL_GetTick() - start_ms) < duration_ms) {
        uint32_t now_ms = HAL_GetTick();
        if ((now_ms - last_ms) < sample_interval_ms) continue;
        last_ms = now_ms;

        for (int i = 0; i < ENC_COUNT; ++i) {
            Encoder_t *e = &g_encoders[i];
            uint16_t now16 = read_cnt16(e->htim);
            int16_t d16 = diff16(now16, e->last_cnt);
            e->last_cnt = now16;
            sum_counts[i] += (int32_t)d16 * (int32_t)e->polarity;
        }
    }

    float dur_s = (float)duration_ms / 1000.0f;
    for (int i = 0; i < ENC_COUNT; ++i) {
        float bias = (dur_s > 0.0f) ? ((float)sum_counts[i] / dur_s) : 0.0f;
        if (fabsf(bias) > ENC_BOOT_CALIB_MAX_BIAS_CPS) {
            s_vel_bias_cps[i] = 0.0f;
            all_ok = 0u;
        } else {
            s_vel_bias_cps[i] = bias;
        }
        g_encoders[i].vel_cps = 0.0f;
        g_encoders[i].last_cnt = read_cnt16(g_encoders[i].htim);
    }

    return all_ok;
}

/* ==================== 断线检测逻辑 ==================== */

static void check_encoder_health(Encoder_t *e, float raw_vel)
{
    float abs_vel = fabsf(raw_vel);
    float abs_motor = fabsf(e->motor_output);

    /* 1. 毛刺检测：速度异常大 */
    if (abs_vel > ENC_GLITCH_THRESHOLD_CPS) {
        e->status = ENC_STATUS_GLITCH;
        e->fault_ticks++;
        e->total_glitches++;
        return;
    }

    /* 2. 断线检测：电机有输出但编码器无响应 */
    if (abs_motor > ENC_MOTOR_THRESHOLD && abs_vel < ENC_DISCONNECT_THRESHOLD_CPS) {
        e->zero_vel_ticks++;
        if (e->zero_vel_ticks >= ENC_DISCONNECT_TICKS) {
            if (e->status != ENC_STATUS_DISCONNECT) {
                e->total_disconnects++;
            }
            e->status = ENC_STATUS_DISCONNECT;
            e->fault_ticks++;
        }
        return;
    }

    /* 3. 恢复检测 */
    if (e->status != ENC_STATUS_OK) {
        /* 需要连续几次正常数据才能恢复 */
        if (e->fault_ticks > 0) {
            e->fault_ticks--;
        }
        if (e->fault_ticks == 0 && abs_vel < ENC_GLITCH_THRESHOLD_CPS) {
            e->status = ENC_STATUS_OK;
            e->zero_vel_ticks = 0;
        }
    } else {
        e->zero_vel_ticks = 0;
        e->fault_ticks = 0;
    }

    /* 4. 更新上次有效速度 */
    if (abs_vel < ENC_GLITCH_THRESHOLD_CPS) {
        e->last_valid_vel = raw_vel;
    }
}

/* ==================== 速度更新 ==================== */

void Encoder_UpdateAll(float dt)
{
    if (dt <= 0.0f) return;

    const float a = (ENC_VEL_LPF_ALPHA >= 0.0f && ENC_VEL_LPF_ALPHA <= 1.0f)
                        ? ENC_VEL_LPF_ALPHA
                        : 0.80f;
    const float cps_limit = 1.5f * MAX_CPS;

    for (int i = 0; i < ENC_COUNT; ++i)
    {
        Encoder_t *e = &g_encoders[i];

        uint16_t now16 = read_cnt16(e->htim);
        int16_t  d16   = diff16(now16, e->last_cnt);
        e->last_cnt    = now16;

        e->pos += (int32_t)d16;

        float raw = ((float)d16 * (float)e->polarity) / dt;
        raw -= s_vel_bias_cps[i];

        /* 健康检测 */
        check_encoder_health(e, raw);

        /* 如果检测到断线，使用上次有效速度或0 */
        if (e->status == ENC_STATUS_DISCONNECT) {
            raw = 0.0f;
        } else if (e->status == ENC_STATUS_GLITCH) {
            raw = e->last_valid_vel;
        }

        /* 毛刺保护 */
        if (fabsf(raw) > cps_limit) {
            raw = 0.0f;
        }

        /* 低通滤波 */
        e->vel_cps = a * e->vel_cps + (1.0f - a) * raw;
    }
}

/* ==================== 断线检测 API ==================== */

void Encoder_SetMotorOutput(encoder_id_t id, float pwm_output)
{
    if (id >= ENC_COUNT) return;
    g_encoders[id].motor_output = pwm_output;
}

EncoderStatus_t Encoder_GetStatus(encoder_id_t id)
{
    if (id >= ENC_COUNT) return ENC_STATUS_OK;
    return g_encoders[id].status;
}

uint8_t Encoder_GetFaultMask(void)
{
    uint8_t mask = 0u;
    for (int i = 0; i < ENC_COUNT; ++i) {
        if (g_encoders[i].status != ENC_STATUS_OK) {
            mask |= (1u << i);
        }
    }
    return mask;
}

const char* Encoder_GetStatusStr(encoder_id_t id)
{
    if (id >= ENC_COUNT) return "INVALID";

    switch (g_encoders[id].status) {
        case ENC_STATUS_OK:         return "OK";
        case ENC_STATUS_DISCONNECT: return "DISCONNECT";
        case ENC_STATUS_GLITCH:     return "GLITCH";
        case ENC_STATUS_RECOVERING: return "RECOVERING";
        default:                    return "UNKNOWN";
    }
}

void Encoder_ResetFault(encoder_id_t id)
{
    if (id >= ENC_COUNT) {
        /* 重置所有 */
        for (int i = 0; i < ENC_COUNT; ++i) {
            g_encoders[i].status = ENC_STATUS_OK;
            g_encoders[i].fault_ticks = 0;
            g_encoders[i].zero_vel_ticks = 0;
        }
    } else {
        g_encoders[id].status = ENC_STATUS_OK;
        g_encoders[id].fault_ticks = 0;
        g_encoders[id].zero_vel_ticks = 0;
    }
}

/* ==================== 调试信息 ==================== */

void Encoder_GetDebugInfo(char *buf, size_t size)
{
    if (!buf || size < 128) return;

    snprintf(buf, size,
             "ENC: L1=%d L2=%d R1=%d R2=%d\r\n",
             (int)__HAL_TIM_GET_COUNTER(&htim2),
             (int)__HAL_TIM_GET_COUNTER(&htim3),
             (int)__HAL_TIM_GET_COUNTER(&htim4),
             (int)__HAL_TIM_GET_COUNTER(&htim5));
}

uint8_t Encoder_VerifyAll(void)
{
    uint8_t all_ok = 1;

    for (int i = 0; i < ENC_COUNT; ++i)
    {
        Encoder_t *e = &g_encoders[i];
        TIM_HandleTypeDef *htim = e->htim;

        uint32_t cr1 = htim->Instance->CR1;
        uint8_t enabled = (cr1 & TIM_CR1_CEN) ? 1 : 0;

        uint32_t smcr = htim->Instance->SMCR;
        uint8_t slave_mode = (smcr & TIM_SMCR_SMS) >> TIM_SMCR_SMS_Pos;

        uint8_t encoder_mode = (slave_mode == 3) ? 1 : 0;

        if (!enabled || !encoder_mode) {
            all_ok = 0;
        }
    }

    return all_ok;
}