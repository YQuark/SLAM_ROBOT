#include "mpu6050.h"

#include <math.h>
#include <stdio.h>

#include "usart.h"

#define DEG2RAD 0.0174532925f
#define RAD2DEG 57.2957795f
#define IMU_ACCEL_BIAS_CALIB_MIN_G 0.85f
#define IMU_ACCEL_BIAS_CALIB_MAX_G 1.15f
#define IMU_ACCEL_CORR_NORM_MIN_G 0.60f
#define IMU_ACCEL_CORR_NORM_MAX_G 1.60f
#define IMU_ACCEL_AXIS_SAT_G      1.95f

static Attitude_t s_attitude = {0};
static IMU_Calib_t s_calib = {0};
static uint8_t s_fusion_mode = IMU_FILTER_DEFAULT;
static float s_mahony_ix = 0.0f;
static float s_mahony_iy = 0.0f;
static float s_mahony_iz = 0.0f;
static IMU_CalibFailReason_t s_calib_fail_reason = IMU_CALIB_FAIL_NONE;
static float s_last_calib_accel_norm = 0.0f;
static float s_last_calib_gyro_abs = 0.0f;

static HAL_StatusTypeDef MPU6050_WriteReg(uint8_t reg, uint8_t value);
static void Attitude_UpdateFromAccel(float ax, float ay, float az, float *roll, float *pitch);
static void EulerToQuaternion(float roll, float pitch, float yaw, float *qw, float *qx, float *qy, float *qz);
static void QuaternionNormalize(float *qw, float *qx, float *qy, float *qz);
static void QuaternionToEuler(float qw, float qx, float qy, float qz, float *roll, float *pitch, float *yaw);
static void RotateBodyToWorldByQuat(float qw, float qx, float qy, float qz,
                                    float bx, float by, float bz,
                                    float *wx, float *wy, float *wz);
static void IntegrateGyroOnly(float gx_dps, float gy_dps, float gz_dps, float dt);
static void MahonyUpdate(float gx_dps, float gy_dps, float gz_dps,
                         float ax, float ay, float az, float dt);

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

    MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x80);
    HAL_Delay(100);
    MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x00);
    HAL_Delay(100);

    status = MPU6050_ReadReg(MPU6050_REG_WHO_AM_I, &check);
    printf(">> MPU6050 WHO_AM_I Read: 0x%02X (Expected: 0x68)\r\n", check);
    if (status != HAL_OK) {
        printf(">> MPU6050 I2C Read Error!\r\n");
        return status;
    }

    status = MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x00);
    if (status != HAL_OK) return status;

    status = MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x07);
    if (status != HAL_OK) return status;

    status = MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x06);
    if (status != HAL_OK) return status;

    status = MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, 0x18);
    if (status != HAL_OK) return status;

    status = MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, 0x00);
    if (status != HAL_OK) return status;

    Attitude_Init();
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadRaw(MPU6050_Data_t *data)
{
    uint8_t buf[14];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(MPU6050_I2C,
                              MPU6050_ADDR,
                              MPU6050_REG_ACCEL_XOUT_H,
                              I2C_MEMADD_SIZE_8BIT,
                              buf,
                              14,
                              100);
    if (status != HAL_OK) {
        return status;
    }

    data->ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
    data->ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
    data->az_raw = (int16_t)((buf[4] << 8) | buf[5]);
    data->temp_raw = (int16_t)((buf[6] << 8) | buf[7]);
    data->gx_raw = (int16_t)((buf[8] << 8) | buf[9]);
    data->gy_raw = (int16_t)((buf[10] << 8) | buf[11]);
    data->gz_raw = (int16_t)((buf[12] << 8) | buf[13]);

    return HAL_OK;
}

void MPU6050_Convert(MPU6050_Data_t *data)
{
    const float accel_sens = 16384.0f;
    const float gyro_sens = 16.4f;

    data->ax_g = data->ax_raw / accel_sens;
    data->ay_g = data->ay_raw / accel_sens;
    data->az_g = data->az_raw / accel_sens;

    data->gx_dps = data->gx_raw / gyro_sens;
    data->gy_dps = data->gy_raw / gyro_sens;
    data->gz_dps = data->gz_raw / gyro_sens;

    data->temp_c = (float)data->temp_raw / 340.0f + 36.53f;
}

void IMU_Print(const MPU6050_Data_t *d)
{
    char buf[200];
    int n = snprintf(buf, sizeof(buf),
                     "ACC[g]: X=%.3f Y=%.3f Z=%.3f | "
                     "GYRO[dps]: X=%.3f Y=%.3f Z=%.3f | "
                     "TEMP=%.2fC\r\n",
                     d->ax_g, d->ay_g, d->az_g,
                     d->gx_dps, d->gy_dps, d->gz_dps,
                     d->temp_c);

    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 100);
}

void IMU_PrintRawDebug(const MPU6050_Data_t *d)
{
    char buf[220];
    int32_t ax_mg;
    int32_t ay_mg;
    int32_t az_mg;
    int32_t gx_cdps;
    int32_t gy_cdps;
    int32_t gz_cdps;
    int32_t accel_norm_mg;
    int n;

    if (d == NULL) {
        return;
    }

    ax_mg = (int32_t)(d->ax_g * 1000.0f);
    ay_mg = (int32_t)(d->ay_g * 1000.0f);
    az_mg = (int32_t)(d->az_g * 1000.0f);
    gx_cdps = (int32_t)(d->gx_dps * 100.0f);
    gy_cdps = (int32_t)(d->gy_dps * 100.0f);
    gz_cdps = (int32_t)(d->gz_dps * 100.0f);
    accel_norm_mg = (int32_t)(sqrtf(d->ax_g * d->ax_g + d->ay_g * d->ay_g + d->az_g * d->az_g) * 1000.0f);
    n = snprintf(buf, sizeof(buf),
                 "IMU RAW ax=%d ay=%d az=%d gx=%d gy=%d gz=%d | "
                 "ACC[mg]=%ld,%ld,%ld | GYRO[cdps]=%ld,%ld,%ld | |a|[mg]=%ld\r\n",
                 d->ax_raw, d->ay_raw, d->az_raw,
                 d->gx_raw, d->gy_raw, d->gz_raw,
                 (long)ax_mg, (long)ay_mg, (long)az_mg,
                 (long)gx_cdps, (long)gy_cdps, (long)gz_cdps,
                 (long)accel_norm_mg);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 100);
}

void Attitude_Init(void)
{
    s_attitude.roll = 0.0f;
    s_attitude.pitch = 0.0f;
    s_attitude.yaw = 0.0f;

    s_attitude.qw = 1.0f;
    s_attitude.qx = 0.0f;
    s_attitude.qy = 0.0f;
    s_attitude.qz = 0.0f;

    s_attitude.gyro_bias_x = 0.0f;
    s_attitude.gyro_bias_y = 0.0f;
    s_attitude.gyro_bias_z = 0.0f;

    s_attitude.accel_bias_x = 0.0f;
    s_attitude.accel_bias_y = 0.0f;
    s_attitude.accel_bias_z = 0.0f;

    s_attitude.linear_ax = 0.0f;
    s_attitude.linear_ay = 0.0f;
    s_attitude.linear_az = 0.0f;
    s_attitude.world_linear_ax = 0.0f;
    s_attitude.world_linear_ay = 0.0f;
    s_attitude.world_linear_az = 0.0f;
    s_attitude.valid = 0u;
    s_attitude.accel_valid = 0u;

    s_calib.state = IMU_CALIB_IDLE;
    s_calib.sample_count = 0u;
    s_calib.elapsed_s = 0.0f;
    s_calib.sum_gx = 0.0f;
    s_calib.sum_gy = 0.0f;
    s_calib.sum_gz = 0.0f;
    s_calib.sum_ax = 0.0f;
    s_calib.sum_ay = 0.0f;
    s_calib.sum_az = 0.0f;
    s_calib.motion_detected = 0u;

    s_mahony_ix = 0.0f;
    s_mahony_iy = 0.0f;
    s_mahony_iz = 0.0f;
    s_fusion_mode = IMU_FILTER_DEFAULT;
    s_calib_fail_reason = IMU_CALIB_FAIL_NONE;
    s_last_calib_accel_norm = 0.0f;
    s_last_calib_gyro_abs = 0.0f;
}

void Attitude_StartCalibration(void)
{
    s_calib.state = IMU_CALIB_RUNNING;
    s_calib.sample_count = 0u;
    s_calib.elapsed_s = 0.0f;
    s_calib.sum_gx = 0.0f;
    s_calib.sum_gy = 0.0f;
    s_calib.sum_gz = 0.0f;
    s_calib.sum_ax = 0.0f;
    s_calib.sum_ay = 0.0f;
    s_calib.sum_az = 0.0f;
    s_calib.motion_detected = 0u;
    s_calib_fail_reason = IMU_CALIB_FAIL_NONE;
    s_last_calib_accel_norm = 0.0f;
    s_last_calib_gyro_abs = 0.0f;
}

IMU_CalibState_t Attitude_GetCalibState(void)
{
    return s_calib.state;
}

IMU_CalibFailReason_t Attitude_GetCalibFailReason(void)
{
    return s_calib_fail_reason;
}

void Attitude_GetCalibDebug(float *accel_norm, float *gyro_abs_sum)
{
    if (accel_norm) {
        *accel_norm = s_last_calib_accel_norm;
    }
    if (gyro_abs_sum) {
        *gyro_abs_sum = s_last_calib_gyro_abs;
    }
}

static void Attitude_UpdateFromAccel(float ax, float ay, float az, float *roll, float *pitch)
{
    *roll = atan2f(ay, az) * RAD2DEG;
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
}

static void EulerToQuaternion(float roll, float pitch, float yaw, float *qw, float *qx, float *qy, float *qz)
{
    float cr = cosf(roll * DEG2RAD * 0.5f);
    float sr = sinf(roll * DEG2RAD * 0.5f);
    float cp = cosf(pitch * DEG2RAD * 0.5f);
    float sp = sinf(pitch * DEG2RAD * 0.5f);
    float cy = cosf(yaw * DEG2RAD * 0.5f);
    float sy = sinf(yaw * DEG2RAD * 0.5f);

    *qw = cr * cp * cy + sr * sp * sy;
    *qx = sr * cp * cy - cr * sp * sy;
    *qy = cr * sp * cy + sr * cp * sy;
    *qz = cr * cp * sy - sr * sp * cy;
}

static void QuaternionNormalize(float *qw, float *qx, float *qy, float *qz)
{
    float norm = sqrtf((*qw) * (*qw) + (*qx) * (*qx) + (*qy) * (*qy) + (*qz) * (*qz));

    if (norm < 1e-6f) {
        *qw = 1.0f;
        *qx = 0.0f;
        *qy = 0.0f;
        *qz = 0.0f;
        return;
    }

    *qw /= norm;
    *qx /= norm;
    *qy /= norm;
    *qz /= norm;
}

static void QuaternionToEuler(float qw, float qx, float qy, float qz, float *roll, float *pitch, float *yaw)
{
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    float sinp = 2.0f * (qw * qy - qz * qx);
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);

    *roll = atan2f(sinr_cosp, cosr_cosp) * RAD2DEG;
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf(90.0f, sinp);
    } else {
        *pitch = asinf(sinp) * RAD2DEG;
    }
    *yaw = atan2f(siny_cosp, cosy_cosp) * RAD2DEG;
}

static void RotateBodyToWorldByQuat(float qw, float qx, float qy, float qz,
                                    float bx, float by, float bz,
                                    float *wx, float *wy, float *wz)
{
    float r11 = 1.0f - 2.0f * (qy * qy + qz * qz);
    float r12 = 2.0f * (qx * qy - qw * qz);
    float r13 = 2.0f * (qx * qz + qw * qy);
    float r21 = 2.0f * (qx * qy + qw * qz);
    float r22 = 1.0f - 2.0f * (qx * qx + qz * qz);
    float r23 = 2.0f * (qy * qz - qw * qx);
    float r31 = 2.0f * (qx * qz - qw * qy);
    float r32 = 2.0f * (qy * qz + qw * qx);
    float r33 = 1.0f - 2.0f * (qx * qx + qy * qy);

    *wx = r11 * bx + r12 * by + r13 * bz;
    *wy = r21 * bx + r22 * by + r23 * bz;
    *wz = r31 * bx + r32 * by + r33 * bz;
}

static void IntegrateGyroOnly(float gx_dps, float gy_dps, float gz_dps, float dt)
{
    float qw = s_attitude.qw;
    float qx = s_attitude.qx;
    float qy = s_attitude.qy;
    float qz = s_attitude.qz;
    float old_qw = qw;
    float old_qx = qx;
    float old_qy = qy;
    float old_qz = qz;
    float gx = gx_dps * DEG2RAD;
    float gy = gy_dps * DEG2RAD;
    float gz = gz_dps * DEG2RAD;
    float half_dt = 0.5f * dt;

    qw += (-old_qx * gx - old_qy * gy - old_qz * gz) * half_dt;
    qx += (old_qw * gx + old_qy * gz - old_qz * gy) * half_dt;
    qy += (old_qw * gy - old_qx * gz + old_qz * gx) * half_dt;
    qz += (old_qw * gz + old_qx * gy - old_qy * gx) * half_dt;

    QuaternionNormalize(&qw, &qx, &qy, &qz);

    s_attitude.qw = qw;
    s_attitude.qx = qx;
    s_attitude.qy = qy;
    s_attitude.qz = qz;
    QuaternionToEuler(qw, qx, qy, qz, &s_attitude.roll, &s_attitude.pitch, &s_attitude.yaw);
}

static void MahonyUpdate(float gx_dps, float gy_dps, float gz_dps,
                         float ax, float ay, float az, float dt)
{
    float norm;
    float vx;
    float vy;
    float vz;
    float ex;
    float ey;
    float ez;
    float gx;
    float gy;
    float gz;
    float half_dt;
    float qw = s_attitude.qw;
    float qx = s_attitude.qx;
    float qy = s_attitude.qy;
    float qz = s_attitude.qz;
    float old_qw = qw;
    float old_qx = qx;
    float old_qy = qy;
    float old_qz = qz;

    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 1e-6f) {
        return;
    }

    ax /= norm;
    ay /= norm;
    az /= norm;

    vx = 2.0f * (qx * qz - qw * qy);
    vy = 2.0f * (qw * qx + qy * qz);
    vz = qw * qw - qx * qx - qy * qy + qz * qz;

    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    s_mahony_ix += IMU_MAHONY_KI * ex * dt;
    s_mahony_iy += IMU_MAHONY_KI * ey * dt;
    s_mahony_iz += IMU_MAHONY_KI * ez * dt;

    gx = gx_dps * DEG2RAD + IMU_MAHONY_KP * ex + s_mahony_ix;
    gy = gy_dps * DEG2RAD + IMU_MAHONY_KP * ey + s_mahony_iy;
    gz = gz_dps * DEG2RAD + IMU_MAHONY_KP * ez + s_mahony_iz;
    half_dt = 0.5f * dt;

    qw += (-old_qx * gx - old_qy * gy - old_qz * gz) * half_dt;
    qx += (old_qw * gx + old_qy * gz - old_qz * gy) * half_dt;
    qy += (old_qw * gy - old_qx * gz + old_qz * gx) * half_dt;
    qz += (old_qw * gz + old_qx * gy - old_qy * gx) * half_dt;

    QuaternionNormalize(&qw, &qx, &qy, &qz);

    s_attitude.qw = qw;
    s_attitude.qx = qx;
    s_attitude.qy = qy;
    s_attitude.qz = qz;
    QuaternionToEuler(qw, qx, qy, qz, &s_attitude.roll, &s_attitude.pitch, &s_attitude.yaw);
}

void Attitude_Update(const MPU6050_Data_t *imu, float dt)
{
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
    float acc_roll;
    float acc_pitch;
    float accel_magnitude;
    float gyro_abs;
    float gravity_x;
    float gravity_y;
    float gravity_z;
    float avg_ax;
    float avg_ay;
    float avg_az;
    float avg_accel_magnitude;
    uint8_t accel_trustworthy;
    uint8_t gyro_bad;

    if (imu == NULL || dt <= 0.0f) {
        s_attitude.valid = 0u;
        return;
    }

    if (s_calib.state == IMU_CALIB_FAILED) {
        s_attitude.valid = 0u;
        s_attitude.linear_ax = 0.0f;
        s_attitude.linear_ay = 0.0f;
        s_attitude.linear_az = 0.0f;
        s_attitude.world_linear_ax = 0.0f;
        s_attitude.world_linear_ay = 0.0f;
        s_attitude.world_linear_az = 0.0f;
        s_attitude.accel_valid = 0u;
        return;
    }

    if (s_calib.state == IMU_CALIB_RUNNING) {
        accel_magnitude = sqrtf(imu->ax_g * imu->ax_g + imu->ay_g * imu->ay_g + imu->az_g * imu->az_g);
        gyro_abs = fabsf(imu->gx_dps) + fabsf(imu->gy_dps) + fabsf(imu->gz_dps);
        s_last_calib_accel_norm = accel_magnitude;
        s_last_calib_gyro_abs = gyro_abs;
        gyro_bad = (gyro_abs > (3.0f * IMU_CALIB_MAX_GYRO_DPS)) ? 1u : 0u;

        if (gyro_bad) {
            s_calib.motion_detected++;
            if (s_calib.motion_detected > 10u) {
                s_calib.state = IMU_CALIB_FAILED;
                s_attitude.valid = 0u;
                s_attitude.accel_valid = 0u;
                s_calib_fail_reason = IMU_CALIB_FAIL_GYRO;
                return;
            }
        } else {
            s_calib.motion_detected = 0u;
        }

        s_calib.sum_gx += imu->gx_dps;
        s_calib.sum_gy += imu->gy_dps;
        s_calib.sum_gz += imu->gz_dps;
        s_calib.sum_ax += imu->ax_g;
        s_calib.sum_ay += imu->ay_g;
        s_calib.sum_az += imu->az_g;
        s_calib.sample_count++;
        s_calib.elapsed_s += dt;

        if (s_calib.elapsed_s >= ((float)IMU_CALIB_DURATION_MS / 1000.0f) &&
            s_calib.sample_count > 0u) {
            s_attitude.gyro_bias_x = s_calib.sum_gx / (float)s_calib.sample_count;
            s_attitude.gyro_bias_y = s_calib.sum_gy / (float)s_calib.sample_count;
            s_attitude.gyro_bias_z = s_calib.sum_gz / (float)s_calib.sample_count;

            avg_ax = s_calib.sum_ax / (float)s_calib.sample_count;
            avg_ay = s_calib.sum_ay / (float)s_calib.sample_count;
            avg_az = s_calib.sum_az / (float)s_calib.sample_count;
            avg_accel_magnitude = sqrtf(avg_ax * avg_ax + avg_ay * avg_ay + avg_az * avg_az);

            if (avg_accel_magnitude >= IMU_ACCEL_BIAS_CALIB_MIN_G &&
                avg_accel_magnitude <= IMU_ACCEL_BIAS_CALIB_MAX_G) {
                s_attitude.accel_bias_x = avg_ax;
                s_attitude.accel_bias_y = avg_ay;
                s_attitude.accel_bias_z = avg_az - 1.0f;
                s_attitude.accel_valid = 1u;
            } else {
                s_attitude.accel_bias_x = 0.0f;
                s_attitude.accel_bias_y = 0.0f;
                s_attitude.accel_bias_z = 0.0f;
                s_attitude.accel_valid = 0u;
            }

            s_calib.state = IMU_CALIB_DONE;
            s_attitude.valid = 1u;
            s_mahony_ix = 0.0f;
            s_mahony_iy = 0.0f;
            s_mahony_iz = 0.0f;
            Attitude_Reset();

            {
                char buf[160];
                int32_t bias_x_cdps = (int32_t)(s_attitude.gyro_bias_x * 100.0f);
                int32_t bias_y_cdps = (int32_t)(s_attitude.gyro_bias_y * 100.0f);
                int32_t bias_z_cdps = (int32_t)(s_attitude.gyro_bias_z * 100.0f);
                int32_t accel_norm_mg = (int32_t)(avg_accel_magnitude * 1000.0f);
                int n = snprintf(buf, sizeof(buf),
                                 "IMU Calib DONE: dt_ms=%lu gyro_bias_cdps=%ld,%ld,%ld accel_ok=%u accel_norm_mg=%ld\r\n",
                                 (unsigned long)(s_calib.elapsed_s * 1000.0f),
                                 (long)bias_x_cdps,
                                 (long)bias_y_cdps,
                                 (long)bias_z_cdps,
                                 (unsigned int)s_attitude.accel_valid,
                                 (long)accel_norm_mg);
                HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 100);
            }
        }
        return;
    }

    gx = imu->gx_dps - s_attitude.gyro_bias_x;
    gy = imu->gy_dps - s_attitude.gyro_bias_y;
    gz = imu->gz_dps - s_attitude.gyro_bias_z;

    ax = imu->ax_g - s_attitude.accel_bias_x;
    ay = imu->ay_g - s_attitude.accel_bias_y;
    az = imu->az_g - s_attitude.accel_bias_z;
    accel_magnitude = sqrtf(ax * ax + ay * ay + az * az);
    accel_trustworthy = s_attitude.accel_valid ? 1u : 0u;
    if (fabsf(ax) >= IMU_ACCEL_AXIS_SAT_G ||
        fabsf(ay) >= IMU_ACCEL_AXIS_SAT_G ||
        fabsf(az) >= IMU_ACCEL_AXIS_SAT_G ||
        accel_magnitude < IMU_ACCEL_CORR_NORM_MIN_G ||
        accel_magnitude > IMU_ACCEL_CORR_NORM_MAX_G) {
        accel_trustworthy = 0u;
    }

    if (s_fusion_mode == IMU_FILTER_COMPLEMENTARY) {
        s_attitude.roll += gx * dt;
        s_attitude.pitch += gy * dt;
        s_attitude.yaw += gz * dt;
        if (accel_trustworthy) {
            Attitude_UpdateFromAccel(ax, ay, az, &acc_roll, &acc_pitch);
            s_attitude.roll = IMU_GYRO_WEIGHT * s_attitude.roll + (1.0f - IMU_GYRO_WEIGHT) * acc_roll;
            s_attitude.pitch = IMU_GYRO_WEIGHT * s_attitude.pitch + (1.0f - IMU_GYRO_WEIGHT) * acc_pitch;
        }

        while (s_attitude.yaw > 180.0f) s_attitude.yaw -= 360.0f;
        while (s_attitude.yaw < -180.0f) s_attitude.yaw += 360.0f;

        EulerToQuaternion(s_attitude.roll, s_attitude.pitch, s_attitude.yaw,
                          &s_attitude.qw, &s_attitude.qx, &s_attitude.qy, &s_attitude.qz);
        QuaternionNormalize(&s_attitude.qw, &s_attitude.qx, &s_attitude.qy, &s_attitude.qz);
    } else {
        if (accel_trustworthy) {
            MahonyUpdate(gx, gy, gz, ax, ay, az, dt);
        } else {
            IntegrateGyroOnly(gx, gy, gz, dt);
        }
    }

    if (!accel_trustworthy) {
        s_attitude.linear_ax = 0.0f;
        s_attitude.linear_ay = 0.0f;
        s_attitude.linear_az = 0.0f;
        s_attitude.world_linear_ax = 0.0f;
        s_attitude.world_linear_ay = 0.0f;
        s_attitude.world_linear_az = 0.0f;
        s_attitude.valid = 1u;
        s_attitude.accel_valid = 0u;
        return;
    }

    gravity_x = 2.0f * (s_attitude.qx * s_attitude.qz - s_attitude.qw * s_attitude.qy);
    gravity_y = 2.0f * (s_attitude.qw * s_attitude.qx + s_attitude.qy * s_attitude.qz);
    gravity_z = s_attitude.qw * s_attitude.qw - s_attitude.qx * s_attitude.qx -
                s_attitude.qy * s_attitude.qy + s_attitude.qz * s_attitude.qz;

    s_attitude.linear_ax = ax - gravity_x;
    s_attitude.linear_ay = ay - gravity_y;
    s_attitude.linear_az = az - gravity_z;

    RotateBodyToWorldByQuat(s_attitude.qw, s_attitude.qx, s_attitude.qy, s_attitude.qz,
                            s_attitude.linear_ax, s_attitude.linear_ay, s_attitude.linear_az,
                            &s_attitude.world_linear_ax,
                            &s_attitude.world_linear_ay,
                            &s_attitude.world_linear_az);

    s_attitude.valid = 1u;
    s_attitude.accel_valid = 1u;
}

const Attitude_t *Attitude_GetData(void)
{
    return &s_attitude;
}

void Attitude_Reset(void)
{
    s_attitude.roll = 0.0f;
    s_attitude.pitch = 0.0f;
    s_attitude.yaw = 0.0f;
    s_attitude.qw = 1.0f;
    s_attitude.qx = 0.0f;
    s_attitude.qy = 0.0f;
    s_attitude.qz = 0.0f;
    s_attitude.linear_ax = 0.0f;
    s_attitude.linear_ay = 0.0f;
    s_attitude.linear_az = 0.0f;
    s_attitude.world_linear_ax = 0.0f;
    s_attitude.world_linear_ay = 0.0f;
    s_attitude.world_linear_az = 0.0f;
}

void Attitude_SetYaw(float yaw)
{
    s_attitude.yaw = yaw;
    while (s_attitude.yaw > 180.0f) s_attitude.yaw -= 360.0f;
    while (s_attitude.yaw < -180.0f) s_attitude.yaw += 360.0f;

    EulerToQuaternion(s_attitude.roll, s_attitude.pitch, s_attitude.yaw,
                      &s_attitude.qw, &s_attitude.qx, &s_attitude.qy, &s_attitude.qz);
    QuaternionNormalize(&s_attitude.qw, &s_attitude.qx, &s_attitude.qy, &s_attitude.qz);
}

void Attitude_SetFusionMode(uint8_t mode)
{
    if (mode == IMU_FILTER_COMPLEMENTARY) {
        s_fusion_mode = IMU_FILTER_COMPLEMENTARY;
    } else {
        s_fusion_mode = IMU_FILTER_MAHONY;
    }
}

uint8_t Attitude_GetFusionMode(void)
{
    return s_fusion_mode;
}

void Attitude_BodyToWorld(float bx, float by, float bz, float *wx, float *wy, float *wz)
{
    RotateBodyToWorldByQuat(s_attitude.qw, s_attitude.qx, s_attitude.qy, s_attitude.qz,
                            bx, by, bz, wx, wy, wz);
}
