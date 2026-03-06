// Created by yushu on 2025/12/8.
#include "encoder.h"
#include <math.h>
#include "robot_config.h"
#include <stdio.h>  // For snprintf

/* 这些句柄在 tim.c 里定义，这里只做 extern 声明 */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

Encoder_t g_encoders[ENC_COUNT] =
{
    { &htim2, 0, 0, 0.0f, +1 },  /* ENC_L1 : TIM2 (32-bit counter) */
    { &htim3, 0, 0, 0.0f, +1 },  /* ENC_L2 : TIM3 (16-bit counter) */
    { &htim4, 0, 0, 0.0f, +1 },  /* ENC_R1 : TIM4 (16-bit counter) */
    { &htim5, 0, 0, 0.0f, +1 },  /* ENC_R2 : TIM5 (32-bit counter) */
};

/* 统一用“低16位差分”做溢出处理：适用于 16 位定时器，也适用于 32 位定时器的低16位。
 * 控制周期较短时，这个差分不会跨越 32768 counts 的半周，因此符号差分可靠。
 */
static inline int16_t diff16(uint16_t now16, uint16_t last16)
{
    return (int16_t)(now16 - last16);
}

static inline uint16_t read_cnt16(TIM_HandleTypeDef *htim)
{
    /* TIM2/TIM5 是 32 位，TIM3/TIM4 是 16 位。
     * 这里统一取低16位，避免你之前那种“先截断再拿截断值当真实计数”的隐患。
     */
    uint32_t c = __HAL_TIM_GET_COUNTER(htim);
    return (uint16_t)(c & 0xFFFFu);
}

void Encoder_Init(void)
{
  static uint8_t debug_once = 0;  /* 只在第一次调用时输出调试信息 */
  
  for (int i = 0; i < ENC_COUNT; ++i)    {
        Encoder_t *e = &g_encoders[i];
        e->last_cnt = 0;
        e->pos      = 0;
        e->vel_cps  = 0.0f;

        __HAL_TIM_SET_COUNTER(e->htim, 0);
        HAL_StatusTypeDef status = HAL_TIM_Encoder_Start(e->htim, TIM_CHANNEL_ALL);

        /* 检查启动状态 */
        if (status != HAL_OK) {
            /* 编码器启动失败 - 通过USART输出警告 */
            extern UART_HandleTypeDef huart1;
            char dbg[64];
            int n = snprintf(dbg, sizeof(dbg), 
                           "Encoder %d START FAIL, status=%d\r\n", i, (int)status);
            HAL_UART_Transmit(&huart1, (uint8_t*)dbg, n, 100);
        }

        /* 确保UDIS位被清除，否则编码器无法正常工作 */
        /* 必须在启动编码器后立即清除，因为HAL库可能会设置这个位 */
        e->htim->Instance->CR1 &= ~TIM_CR1_UDIS;

        /* 读一次当前低16位作为起始点，避免第一帧 diff 异常 */
        e->last_cnt = read_cnt16(e->htim);

        /* 只在第一次调用时输出调试信息（通过静态变量） */
        if (!debug_once) {
            /* 这里不能直接输出，需要外部调用 */
            debug_once = 1;
        }
    }
}

void Encoder_UpdateAll(float dt)
{
    if (dt <= 0.0f) return;

    const float a = (ENC_VEL_LPF_ALPHA >= 0.0f && ENC_VEL_LPF_ALPHA <= 1.0f)
                        ? ENC_VEL_LPF_ALPHA
                        : 0.80f;           /* fallback if config is out of range */
    const float cps_limit = 1.5f * MAX_CPS;/* 物理限幅：毛刺丢弃 */

    for (int i = 0; i < ENC_COUNT; ++i)
    {
        Encoder_t *e = &g_encoders[i];

        uint16_t now16 = read_cnt16(e->htim);
        int16_t  d16   = diff16(now16, e->last_cnt);
        e->last_cnt    = now16;

        e->pos += (int32_t)d16;

        float raw = ((float)d16 * (float)e->polarity) / dt;

        /* 毛刺保护：速度离谱直接丢弃 */
        if (fabsf(raw) > cps_limit) {
            raw = 0.0f;
        }

        /* 低通滤波 */
        e->vel_cps = a * e->vel_cps + (1.0f - a) * raw;
    }
}

/* 获取编码器调试信息 */
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

/* 检查编码器GPIO是否正确配置 */
void Encoder_CheckGPIO(void)
{
    /* 通过读取GPIO IDR寄存器来检查引脚状态 */
    /* 注意：这只是检查引脚的输入状态，不能确认引脚是否配置为编码器模式 */
    
    /* 读取引脚状态 */
    uint16_t pc_idr = GPIOC->IDR;  /* PC6, PC7 */
    uint16_t pd_idr = GPIOD->IDR;  /* PD12, PD13 */
    uint16_t pa_idr = GPIOA->IDR;  /* PA0, PA1, PA15 */
    uint16_t pb_idr = GPIOB->IDR;  /* PB3 */
    (void)pc_idr;
    (void)pd_idr;
    (void)pa_idr;
    (void)pb_idr;
    
    /* 检查编码器引脚是否有信号 */
    /* 这需要用户手动转动轮子来观察变化 */
}

/* 验证编码器定时器是否正常工作 */
uint8_t Encoder_VerifyAll(void)
{
    uint8_t all_ok = 1;
    
    /* 检查每个编码器的定时器状态 */
    for (int i = 0; i < ENC_COUNT; ++i)
    {
        Encoder_t *e = &g_encoders[i];
        TIM_HandleTypeDef *htim = e->htim;
        
        /* 读取CR1寄存器检查定时器是否使能 */
        uint32_t cr1 = htim->Instance->CR1;
        uint8_t enabled = (cr1 & TIM_CR1_CEN) ? 1 : 0;
        
        /* 读取SMCR寄存器检查从模式配置 */
        uint32_t smcr = htim->Instance->SMCR;
        uint8_t slave_mode = (smcr & TIM_SMCR_SMS) >> TIM_SMCR_SMS_Pos;
        
        /* 编码器模式应该是SMS=3 (Encoder mode 3) */
        uint8_t encoder_mode = (slave_mode == 3) ? 1 : 0;
        
        if (!enabled || !encoder_mode) {
            all_ok = 0;
        }
    }
    
    return all_ok;
}
