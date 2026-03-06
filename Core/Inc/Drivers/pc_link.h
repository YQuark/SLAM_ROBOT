#ifndef __PC_LINK_H__
#define __PC_LINK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "battery.h"
#include "mpu6050.h"

void PC_Link_Init(UART_HandleTypeDef *huart);

void PC_Link_Poll(const BatteryStatus_t *batt,
                  const MPU6050_Data_t *imu);

void PC_Link_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void PC_Link_UART_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
