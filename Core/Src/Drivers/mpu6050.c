
#include "mpu6050.h"

#include <stdio.h>

#include "usart.h"

// 内部写寄存器（只在本文件用，所以 static）
static HAL_StatusTypeDef MPU6050_WriteReg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(MPU6050_I2C,
                             MPU6050_ADDR,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1,
                             100);
}

// 对外可见的读寄存器（在 main.c 里要用）
HAL_StatusTypeDef MPU6050_ReadReg(uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(MPU6050_I2C,
                            MPU6050_ADDR,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            value,
                            1,
                            100);
}

HAL_StatusTypeDef MPU6050_Init(void)
{
    uint8_t check = 0;
    HAL_StatusTypeDef status;

    // 1. 先尝试复位一下设备（可选，增加稳定性）
    MPU6050_WriteReg(0x6B, 0x80);
    HAL_Delay(100);
    MPU6050_WriteReg(0x6B, 0x00); // 唤醒
    HAL_Delay(100);

    // 2. 读取 WHO_AM_I 寄存器 (0x75)
    status = MPU6050_ReadReg(MPU6050_REG_WHO_AM_I, &check);

    // [调试] 打印读到的 ID，看看究竟是啥
    printf(">> MPU6050 WHO_AM_I Read: 0x%02X (Expected: 0x68)\r\n", check);

    if (status != HAL_OK) {
        printf(">> MPU6050 I2C Read Error!\r\n");
        return status;
    }

    // 3. 不做强制 ID 校验：兼容非原厂/兼容芯片

    status = MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x00);
    if (status != HAL_OK) return status;

    status = MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x07);
    if (status != HAL_OK) return status;

    status = MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x06);
    if (status != HAL_OK) return status;
    status = MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, 0x18);  // +-2000 dps
    if (status != HAL_OK) return status;
    status = MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, 0x00); // +-2g, self-test off
    if (status != HAL_OK) return status;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_Data_t *data)
{
    uint8_t buf[14];
    HAL_StatusTypeDef status;

    // 一次性读出：Accel(6) + Temp(2) + Gyro(6)
    status = HAL_I2C_Mem_Read(MPU6050_I2C,
                              MPU6050_ADDR,
                              MPU6050_REG_ACCEL_XOUT_H,
                              I2C_MEMADD_SIZE_8BIT,
                              buf,
                              14,
                              100);
    if (status != HAL_OK)
        return status;

    data->ax_raw   = (int16_t)((buf[0] << 8) | buf[1]);
    data->ay_raw   = (int16_t)((buf[2] << 8) | buf[3]);
    data->az_raw   = (int16_t)((buf[4] << 8) | buf[5]);
    data->temp_raw = (int16_t)((buf[6] << 8) | buf[7]);
    data->gx_raw   = (int16_t)((buf[8] << 8) | buf[9]);
    data->gy_raw   = (int16_t)((buf[10] << 8) | buf[11]);
    data->gz_raw   = (int16_t)((buf[12] << 8) | buf[13]);

    return HAL_OK;
}

void MPU6050_Convert(MPU6050_Data_t *data)
{
    // 对应 ±2g / ±2000 dps 的灵敏度
    const float ACCEL_SENS = 16384.0f; // LSB / g
    const float GYRO_SENS  = 16.4f;   // LSB / (°/s)

    data->ax_g = data->ax_raw / ACCEL_SENS;
    data->ay_g = data->ay_raw / ACCEL_SENS;
    data->az_g = data->az_raw / ACCEL_SENS;

    data->gx_dps = data->gx_raw / GYRO_SENS;
    data->gy_dps = data->gy_raw / GYRO_SENS;
    data->gz_dps = data->gz_raw / GYRO_SENS;

    // 温度，datasheet：Temp(°C) = raw / 340 + 36.53
    data->temp_c = (float)data->temp_raw / 340.0f + 36.53f;
}

void IMU_Print(const MPU6050_Data_t* d)
{
    char buf[200];
    int n = snprintf(buf, sizeof(buf),
                     "ACC[g]: X=%.3f Y=%.3f Z=%.3f | "
                     "GYRO[dps]: X=%.3f Y=%.3f Z=%.3f | "
                     "TEMP=%.2fC\r\n",
                     d->ax_g, d->ay_g, d->az_g,
                     d->gx_dps, d->gy_dps, d->gz_dps,
                     d->temp_c);

    HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, 100);
}
