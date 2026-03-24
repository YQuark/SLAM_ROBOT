#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

#define MPU6050_I2C_HANDLE   hi2c1
#define MPU6050_I2C          (&MPU6050_I2C_HANDLE)

#define MPU6050_ADDR         (0x68u << 1)

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

#define IMU_GYRO_WEIGHT          0.98f
#define IMU_CALIB_DURATION_MS    3000u
#define IMU_CALIB_MAX_MOTION     0.08f
#define IMU_CALIB_MAX_GYRO_DPS   3.0f
#define IMU_FILTER_COMPLEMENTARY 0u
#define IMU_FILTER_MAHONY        1u
#define IMU_FILTER_DEFAULT       IMU_FILTER_MAHONY
#define IMU_MAHONY_KP            2.0f
#define IMU_MAHONY_KI            0.05f

typedef struct {
    float roll;
    float pitch;
    float yaw;

    float qw;
    float qx;
    float qy;
    float qz;

    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;

    float accel_bias_x;
    float accel_bias_y;
    float accel_bias_z;

    float linear_ax;
    float linear_ay;
    float linear_az;

    float world_linear_ax;
    float world_linear_ay;
    float world_linear_az;

    uint8_t valid;
    uint8_t accel_valid;
} Attitude_t;

typedef enum {
    IMU_CALIB_IDLE = 0,
    IMU_CALIB_RUNNING,
    IMU_CALIB_DONE,
    IMU_CALIB_FAILED
} IMU_CalibState_t;

typedef enum {
    IMU_CALIB_FAIL_NONE = 0,
    IMU_CALIB_FAIL_ACCEL = 1,
    IMU_CALIB_FAIL_GYRO = 2,
    IMU_CALIB_FAIL_BOTH = 3
} IMU_CalibFailReason_t;

typedef struct {
    IMU_CalibState_t state;
    uint32_t sample_count;
    float elapsed_s;
    float sum_gx;
    float sum_gy;
    float sum_gz;
    float sum_ax;
    float sum_ay;
    float sum_az;
    uint8_t motion_detected;
} IMU_Calib_t;

HAL_StatusTypeDef MPU6050_Init(void);
HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_Data_t *data);
void MPU6050_Convert(MPU6050_Data_t *data);
void IMU_Print(const MPU6050_Data_t *d);
void IMU_PrintRawDebug(const MPU6050_Data_t *d);
HAL_StatusTypeDef MPU6050_ReadReg(uint8_t reg, uint8_t *value);

void Attitude_Init(void);
void Attitude_StartCalibration(void);
IMU_CalibState_t Attitude_GetCalibState(void);
IMU_CalibFailReason_t Attitude_GetCalibFailReason(void);
void Attitude_GetCalibDebug(float *accel_norm, float *gyro_abs_sum);
void Attitude_Update(const MPU6050_Data_t *imu, float dt);
const Attitude_t *Attitude_GetData(void);
void Attitude_Reset(void);
void Attitude_SetYaw(float yaw);
void Attitude_SetFusionMode(uint8_t mode);
uint8_t Attitude_GetFusionMode(void);
void Attitude_BodyToWorld(float bx, float by, float bz,
                          float *wx, float *wy, float *wz);

#endif
