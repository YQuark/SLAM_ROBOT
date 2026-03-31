//
// Created by yushu on 2025/11/21.
//
#ifndef __BATTERY_H__
#define __BATTERY_H__

#include "stm32f4xx_hal.h"

typedef struct {
    float voltage;   // 实测电压
    float percent;   // 粗略电量百分比 0~100
} BatteryStatus_t;

void Battery_Init(ADC_HandleTypeDef *hadc);
void Battery_Update(BatteryStatus_t *st);

#endif