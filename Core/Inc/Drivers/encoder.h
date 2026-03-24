//
// Created by yushu on 2025/12/8.
//
#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stddef.h>

/* 编码器方向极性（+1 保持，-1 取反），用于让"车体前进"对应正速度。 */
#ifndef ENC_L1_POLARITY
#define ENC_L1_POLARITY (+1)
#endif
#ifndef ENC_L2_POLARITY
#define ENC_L2_POLARITY (+1)
#endif
#ifndef ENC_R1_POLARITY
#define ENC_R1_POLARITY (-1)
#endif
#ifndef ENC_R2_POLARITY
#define ENC_R2_POLARITY (-1)
#endif

/* ==================== 断线检测参数 ==================== */
#define ENC_DISCONNECT_THRESHOLD_CPS   50.0f   /* 速度低于此值认为静止 */
#define ENC_DISCONNECT_TICKS           100u    /* 连续静止tick数判定断线 (100*20ms=2s) */
#define ENC_GLITCH_THRESHOLD_CPS       20000.0f /* 速度超过此值认为是毛刺/故障 */
#define ENC_MOTOR_THRESHOLD            0.10f   /* PWM输出大于此值认为电机在工作 */

/* ==================== 编码器健康状态 ==================== */
typedef enum {
    ENC_STATUS_OK = 0,          /* 正常 */
    ENC_STATUS_DISCONNECT,      /* 断线/无信号 */
    ENC_STATUS_GLITCH,          /* 信号毛刺/异常 */
    ENC_STATUS_RECOVERING       /* 恢复中 */
} EncoderStatus_t;

/* ==================== 单个编码器状态 ==================== */
typedef struct
{
    TIM_HandleTypeDef *htim;     /* 使用的定时器 */
    uint16_t           last_cnt; /* 上一次的计数值（16bit） */
    int32_t            pos;      /* 累计位置，单位：counts */
    float              vel_cps;  /* 速度，单位：counts/s */
    int8_t             polarity; /* +1 正方向，-1 反方向 */

    /* ===== 断线检测相关 ===== */
    EncoderStatus_t    status;        /* 健康状态 */
    uint16_t           fault_ticks;   /* 连续故障tick计数 */
    uint16_t           zero_vel_ticks;/* 连续零速tick计数 */
    uint32_t           total_glitches;/* 累计毛刺次数 */
    uint32_t           total_disconnects; /* 累计断线次数 */
    float              last_valid_vel;    /* 上次有效速度 */
    float              motor_output;      /* 对应电机的PWM输出 */
} Encoder_t;

/* ==================== 4 路编码器 ID ==================== */
typedef enum
{
    ENC_L1 = 0,
    ENC_L2 = 1,
    ENC_R1 = 2,
    ENC_R2 = 3,
    ENC_COUNT = 4
} encoder_id_t;

/* ==================== 全局编码器数组 ==================== */
extern Encoder_t g_encoders[ENC_COUNT];

/* ==================== 基础 API ==================== */

/**
 * @brief 初始化编码器模块
 */
void Encoder_Init(void);

/**
 * @brief 更新所有编码器速度 (每个控制周期调用)
 * @param dt 采样周期 (秒)
 */
void Encoder_UpdateAll(float dt);

/**
 * @brief 开机静止校准
 */
uint8_t Encoder_CalibrateStatic(uint32_t duration_ms, uint32_t sample_interval_ms);

/**
 * @brief 获取调试信息
 */
void Encoder_GetDebugInfo(char *buf, size_t size);

/* ==================== 断线检测 API ==================== */

/**
 * @brief 设置电机PWM输出状态 (供断线检测使用)
 * @param id 编码器ID
 * @param pwm_output 当前PWM输出值 (-1.0 ~ 1.0)
 */
void Encoder_SetMotorOutput(encoder_id_t id, float pwm_output);

/**
 * @brief 检查编码器健康状态
 * @param id 编码器ID
 * @return 健康状态
 */
EncoderStatus_t Encoder_GetStatus(encoder_id_t id);

/**
 * @brief 检查是否有编码器故障
 * @return 故障掩码 (bit0=L1, bit1=L2, bit2=R1, bit3=R2)
 */
uint8_t Encoder_GetFaultMask(void);

/**
 * @brief 获取故障描述字符串
 * @param id 编码器ID
 * @return 故障描述
 */
const char* Encoder_GetStatusStr(encoder_id_t id);

/**
 * @brief 重置编码器故障状态
 * @param id 编码器ID (ENC_COUNT表示重置所有)
 */
void Encoder_ResetFault(encoder_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
