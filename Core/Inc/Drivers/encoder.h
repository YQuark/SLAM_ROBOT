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

/* 编码器方向极性（+1 保持，-1 取反），用于让“车体前进”对应正速度。 */
#ifndef ENC_L1_POLARITY
#define ENC_L1_POLARITY (+1)
#endif
#ifndef ENC_L2_POLARITY
#define ENC_L2_POLARITY (-1)
#endif
#ifndef ENC_R1_POLARITY
#define ENC_R1_POLARITY (-1)
#endif
#ifndef ENC_R2_POLARITY
#define ENC_R2_POLARITY (+1)
#endif

    /* 单个编码器状态 */
    typedef struct
    {
        TIM_HandleTypeDef *htim;     // 使用的定时器
        uint16_t           last_cnt; // 上一次的计数值（16bit）
        int32_t            pos;      // 累计位置，单位：counts
        float              vel_cps;  // 速度，单位：counts/s
        int8_t             polarity; // +1 正方向，-1 反方向
    } Encoder_t;

    /* 4 路编码器 ID，对应四个电机 */
    typedef enum
    {
        ENC_L1 = 0,
        ENC_L2 = 1,
        ENC_R1 = 2,
        ENC_R2 = 3,
        ENC_COUNT = 4
    } encoder_id_t;

    /* 全局编码器数组 */
    extern Encoder_t g_encoders[ENC_COUNT];

    /* 初始化：清零计数器并启动编码器模式 */
    void Encoder_Init(void);

    /* 每个控制周期调用一次，dt 为秒，内部更新 pos 和 vel_cps */
    void Encoder_UpdateAll(float dt);

    /* 开机静止校准：估计零速偏置（counts/s）并写入补偿 */
    uint8_t Encoder_CalibrateStatic(uint32_t duration_ms, uint32_t sample_interval_ms);

    /* 获取编码器调试信息 */
    void Encoder_GetDebugInfo(char *buf, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
