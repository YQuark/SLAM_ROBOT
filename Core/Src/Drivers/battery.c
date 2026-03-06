//
// Created by yushu on 2025/11/21.
//
#include "battery.h"
#include <stdio.h>

extern UART_HandleTypeDef huart1;   // 为了串口打印

static ADC_HandleTypeDef *s_hadc = NULL;

#define ADC_REF_VOLT  3.3f
#define ADC_MAX       4095.0f

// 根据你的分压电阻修改这里
// 你的分压电阻
#define R_UP   18000.0f
#define R_DOWN 10000.0f
#define DIVIDER_GAIN ((R_UP + R_DOWN) / R_DOWN)

// 电池满电 / 放电终止电压（自己定策略）
#define BATT_FULL_V   8.4f
#define BATT_EMPTY_V  6.0f

#ifndef BATT_DEBUG_ENABLE
#define BATT_DEBUG_ENABLE 0
#endif

void Battery_Init(ADC_HandleTypeDef *hadc)
{
    s_hadc = hadc;
}

static float adc_to_voltage(uint32_t raw)
{
    // 防止任何整数除法的溢出：先转 float，再计算
    float vadc = ((float)raw * ADC_REF_VOLT) / ADC_MAX;  // ADC 脚电压
    float vbatt = vadc * DIVIDER_GAIN;                   // 电池电压

    return vbatt;
}


static float voltage_to_percent(float v)
{
    if (v <= BATT_EMPTY_V) return 0.0f;
    if (v >= BATT_FULL_V)  return 100.0f;

    return (v - BATT_EMPTY_V) * 100.0f / (BATT_FULL_V - BATT_EMPTY_V);
}

void Battery_Update(BatteryStatus_t *st)
{
    if (!s_hadc || !st) return;

    HAL_ADC_Start(s_hadc);
    if (HAL_ADC_PollForConversion(s_hadc, 10) == HAL_OK)
    {
        uint32_t raw = HAL_ADC_GetValue(s_hadc);

        float vbatt  = adc_to_voltage(raw);

        st->voltage  = vbatt;
        st->percent  = voltage_to_percent(vbatt);

#if BATT_DEBUG_ENABLE
        float vadc = ((float)raw * ADC_REF_VOLT) / ADC_MAX;
        static uint8_t dbg_div = 0;
        if (++dbg_div >= 10) {
            dbg_div = 0;
            char dbg[96];
            int n = snprintf(dbg, sizeof(dbg),
                             "ADC raw=%lu, vadc=%.3fV, vbatt=%.3fV\r\n",
                             (unsigned long)raw, vadc, vbatt);
            HAL_UART_Transmit(&huart1, (uint8_t*)dbg, n, 100);
        }
#endif
    }
    HAL_ADC_Stop(s_hadc);
}
