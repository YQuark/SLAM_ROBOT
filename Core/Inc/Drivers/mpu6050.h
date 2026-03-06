#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "stm32f4xx_hal.h"   // 提供 I2C_HandleTypeDef 等类型

// 这里声明 hi2c1 是外部变量，真正定义在 main.c 中
extern I2C_HandleTypeDef hi2c1;

// 把驱动默认绑到 hi2c1 上
#define MPU6050_I2C_HANDLE   hi2c1
#define MPU6050_I2C          (&MPU6050_I2C_HANDLE)

// 器件地址（AD0=0）
#define MPU6050_ADDR         (0x68u << 1)

// 寄存器
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_INT_ENABLE   0x38
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_TEMP_OUT_H   0x41
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75

typedef struct
{
    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;
    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;
    int16_t temp_raw;

    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
    float temp_c;
} MPU6050_Data_t;

// 对外 API
HAL_StatusTypeDef MPU6050_Init(void);
HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_Data_t *data);
void MPU6050_Convert(MPU6050_Data_t *data);
void IMU_Print(const MPU6050_Data_t* d);

// 如果你想在 main 里读寄存器状态，就需要这个原型，且 .c 里不能 static
HAL_StatusTypeDef MPU6050_ReadReg(uint8_t reg, uint8_t *value);

#endif // __MPU6050_H__