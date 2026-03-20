//
// Created by yushu on 2025/12/15.
//
#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "mpu6050.h"

    /* 指令来源 */
    typedef enum {
        CMD_SRC_NONE = 0,
        CMD_SRC_PS2  = 1,
        CMD_SRC_ESP  = 2,
        CMD_SRC_PC   = 3,
    } cmd_source_t;

    /* 运行时控制状态（用于调参观测/上位机） */
    typedef struct {
        float v_cmd;          /* [-1,1] */
        float w_cmd;          /* [-1,1] */
        cmd_source_t src;

        /* IMU 状态 */
        uint8_t imu_enabled;  /* 是否参与控制 */
        uint8_t imu_valid;    /* 数据是否可信 */
        uint8_t imu_err_cnt;  /* 连续错误计数 */

        /* 姿态数据 (来自姿态融合) */
        float roll;           /* 横滚角 (度) */
        float pitch;          /* 俯仰角 (度) */
        float yaw;            /* 航向角 (度) */

        /* PI 内部观测（四轮） */
        float ref_cps[4];
        float meas_cps[4];
        float u_out[4];

        float v_ref_filt;
        float w_ref_filt;
        float v_est;
        float w_est;
        float yaw_est;
        uint8_t mode_transition_active;
        uint8_t src_transition_active;

        /* 编码器健康状态 */
        uint8_t enc_fault_mask;  /* 故障掩码 (bit0=L1, bit1=L2, bit2=R1, bit3=R2) */
    } RobotControlState_t;

    typedef enum {
        MODE_IDLE = 0,    // 怠速 (安全)
        MODE_OPEN_LOOP,   // 开环 (测试电机方向/最大速度)
        MODE_CLOSED_LOOP  // 闭环 (正常 PID)
    } ControlMode_t;


    // 新增接口
    void RobotControl_SetMode(ControlMode_t mode);
    ControlMode_t RobotControl_GetMode(void);

    // 开环专用设置接口
    void RobotControl_SetOpenLoopCmd(float v, float w);

    /* 初始化（会重置 PI、命令、IMU 状态） */
    void RobotControl_Init(void);

    /* 运行期设置：来自 PS2 的 v,w */
    void RobotControl_SetCmd_PS2(float v, float w, uint32_t now_ms);

    /* 运行期设置：来自 ESP 的 v,w */
    void RobotControl_SetCmd_ESP(float v, float w, uint32_t now_ms);
    /* 运行期设置：来自 PC 的 v,w */
    void RobotControl_SetCmd_PC(float v, float w, uint32_t now_ms);

    /* IMU 数据输入：main 在固定周期读取 IMU 后喂给控制模块 */
    void RobotControl_UpdateIMU(const MPU6050_Data_t *imu, uint8_t ok, uint32_t now_ms);

    /* 切换 IMU 参与控制（用于调参时一键禁用） */
    void RobotControl_SetIMUEnabled(uint8_t en);

    /* 速度闭环更新：dt 秒（建议固定 0.02s），内部调用 motor_set 输出 */
    void RobotControl_Update(float dt, uint32_t now_ms);

    /* 读取当前状态 */
    const RobotControlState_t* RobotControl_GetState(void);

#ifdef __cplusplus
}
#endif
#endif /* ROBOT_CONTROL_H */
