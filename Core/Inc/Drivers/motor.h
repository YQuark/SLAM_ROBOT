#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* =================== 用户配置区（按你的工程改） =================== */
/* PWM 定时器与通道映射（默认 TIM1: CH1=L1, CH2=L2, CH3=R1, CH4=R2） */
extern TIM_HandleTypeDef htim1;
#define MOTOR_PWM_TIM                 (&htim1)
#define MOTOR_L1_PWM_CH              TIM_CHANNEL_1
#define MOTOR_L2_PWM_CH              TIM_CHANNEL_2
#define MOTOR_R1_PWM_CH              TIM_CHANNEL_3
#define MOTOR_R2_PWM_CH              TIM_CHANNEL_4

/* DIR 引脚（IN2=DIR），把下列端口/引脚改成你 .ioc 里的实际口径 */
#define MOTOR_L1_DIR_PORT            GPIOE
#define MOTOR_L1_DIR_PIN             GPIO_PIN_8
#define MOTOR_L2_DIR_PORT            GPIOE
#define MOTOR_L2_DIR_PIN             GPIO_PIN_10
#define MOTOR_R1_DIR_PORT            GPIOE
#define MOTOR_R1_DIR_PIN             GPIO_PIN_12
#define MOTOR_R2_DIR_PORT            GPIOE
#define MOTOR_R2_DIR_PIN             GPIO_PIN_15

/* 方向极性（+1 保持，-1 取反）；如果某个轮子不转/反转异常，先改这里再测 */
#ifndef MOTOR_L1_DIR_POLARITY
#define MOTOR_L1_DIR_POLARITY        (+1)
#endif
#ifndef MOTOR_L2_DIR_POLARITY
#define MOTOR_L2_DIR_POLARITY        (-1)
#endif
#ifndef MOTOR_R1_DIR_POLARITY
#define MOTOR_R1_DIR_POLARITY        (-1)
#endif
#ifndef MOTOR_R2_DIR_POLARITY
#define MOTOR_R2_DIR_POLARITY        (+1)
#endif

/* 占空比上限（0..1），先保守一些；闭环稳定后可放开到 1.0f */
#ifndef MOTOR_DUTY_LIMIT_DEFAULT
#define MOTOR_DUTY_LIMIT_DEFAULT     1.0f
#endif
/* ================================================================ */

/* 四轮标识 */
typedef enum {
    MOTOR_L1 = 0,
    MOTOR_L2 = 1,
    MOTOR_R1 = 2,
    MOTOR_R2 = 3
} motor_id_t;

/* 生命周期 */
void motor_init(void);                    // 启动 4 路 PWM，默认空转
void motor_deinit(void);                  // 停止 PWM（可选）

/* 设定速度：-1..1（符号=方向，幅值=占空） */
void motor_set(motor_id_t id, float speed);
static inline void motor_set_i(motor_id_t id, int16_t spd1000){
    if (spd1000 > 1000) spd1000 = 1000;
    if (spd1000 < -1000) spd1000 = -1000;
    motor_set(id, spd1000 / 1000.0f);
}

/* 双轮差速：v 前进(-1..1)，w 旋转(-1..1)，左右= v±w 并限幅 */
void motor_set_diff(float v, float w);

/* 全部空转/刹车 */
void motor_coast_all(void);
void motor_brake_all(void);

/* 单轮极性翻转（+1 正常，-1 反向），用于线序反了无需改线 */
void motor_set_polarity(motor_id_t id, int8_t polarity);

/* 全局占空上限（0..1） */
void motor_set_duty_limit(float limit);

/* 读回当前占空上限 */
float motor_get_duty_limit(void);
void motor_get_debug(uint16_t ccr[4], uint8_t dir[4]);

#ifdef __cplusplus
}
#endif
#endif /* MOTOR_H */
