//
// Created by yushu on 2025/12/15.
//

#pragma once

#include <stdint.h>

/* ======== 周期调度（ms） ======== */
#define CTRL_PERIOD_MS          20u   /* 速度闭环控制周期 */
#define CTRL_ISR_TICK_MS         1u   /* TIM6/HAL tick derived scheduler quantum */
#define PS2_SCAN_INTERVAL_MS    50u
#define PS2_AXIS_DEADZONE      0.05f
#define PS2_V_AXIS_INVERT         1u
#define PS2_W_AXIS_INVERT         1u
#define PS2_V_SCALE             1.0f
#define PS2_W_SCALE             1.0f
#define PS2_DPAD_V             0.72f
#define PS2_DPAD_W             0.58f
#define IMU_READ_INTERVAL_MS    20u
#define BATTERY_INTERVAL_MS    500u
#define OLED_INTERVAL_MS       500u
#define ESP_POLL_INTERVAL_MS    10u
#define PC_POLL_INTERVAL_MS     10u
#define HEARTBEAT_INTERVAL_MS  500u

/* ======== Safety: Watchdog ======== */
#define IWDG_ENABLE                 1u
#define IWDG_TIMEOUT_MS          1200u
#define WDG_CTRL_MAX_LAG_MS       200u
#define WDG_IMU_MAX_LAG_MS        700u
#define WDG_BATT_MAX_LAG_MS      2200u
#define WDG_LINK_MAX_LAG_MS      1000u

/* ======== Safety: Battery protection ======== */
#define BATT_PROTECT_ENABLE          1u
#define BATT_LIMIT_VOLTAGE         6.8f
#define BATT_LIMIT_RECOVER_V       7.0f
#define BATT_CUTOFF_VOLTAGE        6.3f
#define BATT_CUTOFF_RECOVER_V      6.6f
#define BATT_LIMIT_DUTY            0.45f

/* 功能开关 */
#define PS2_ENABLE               1u
#define SELFTEST_ENABLE          0u
#define SELFTEST_SIGNAL_CHECK    1u
#define SELFTEST_ROTATE_MS    1000u

/* 调试输出 */
#define CTRL_DEBUG_ENABLE         0u
#define CTRL_DEBUG_INTERVAL_MS  200u
#define ESP_DEBUG_ENABLE          0u

/* ======== 编码器速度滤波 ======== */
/* 一阶 IIR：vel = a*vel + (1-a)*raw，a 越大越平滑、延迟越大 */
#define ENC_VEL_LPF_ALPHA      0.80f
#define ENC_BOOT_CALIB_ENABLE     1u
#define ENC_BOOT_CALIB_MS       800u
#define ENC_BOOT_CALIB_SAMPLE_MS   5u
#define ENC_BOOT_CALIB_MAX_BIAS_CPS 120.0f

/* ======== 速度标定 ======== */
/* 归一化目标速度 1.0 对应的 counts/s（用于把 v,w 映射到编码器目标） */
#define MAX_CPS               13800.0f

/* ======== 四轮 PI 参数（初值，实车调） ======== */
#define PI_INT_LIM              1.0f

#define KP_L1                0.00028f
#define KI_L1                0.00003f
#define KP_L2                0.00029f
#define KI_L2                0.00003f
#define KP_R1                0.00029f
#define KI_R1                0.00003f
#define KP_R2                0.00029f
#define KI_R2                0.00003f

/* ======== Control optimization toggles ======== */
#define CTRL_USE_DUAL_LOOP           0u
#define CTRL_USE_YAW_HOLD            1u
#define CTRL_USE_AW                  1u
#define CTRL_USE_STATIC_FF           1u

/* ======== Command smoothing / transitions ======== */
#define V_SLEW_MAX                4.0f      /* unit/s in normalized command */
#define W_SLEW_MAX                5.0f      /* unit/s in normalized command */
#define U_SLEW_MAX               15.0f      /* duty/s per wheel output */
#define MODE_TRANSITION_MS         260u
#define MODE_TRANSITION_MAX_CMD   1.00f     /* 满速运行 */
#define SRC_SWITCH_SMOOTH_MS       120u

/* ======== Outer loop (body domain) ======== */
#define OUTER_V_KP               0.60f
#define OUTER_V_KI               0.20f
#define OUTER_W_KP               0.70f
#define OUTER_W_KI               0.22f
#define OUTER_INT_LIM            0.40f

/* ======== IMU / yaw fusion ======== */
#define YAW_FUSION_ALPHA         0.25f      /* 0..1, higher=encoder dominated */
#define YAW_ENC_DPS_SCALE      180.0f      /* encoder-derived normalized w_est to deg/s */
#define IMU_W_DPS_SCALE         180.0f      /* dps that maps to w=1.0 */
#define YAW_HOLD_KP              1.20f
#define YAW_HOLD_KI              0.70f
#define YAW_HOLD_I_LIM           0.18f
#define YAW_HOLD_W_THRESH        0.06f
#define YAW_HOLD_V_MIN           0.08f
#define YAW_HOLD_W_LIM           0.28f

/* ======== Straight-line balance ======== */
/* 直行时根据左右编码器速度差做小幅差速补偿，抑制底盘天然跑偏。 */
#define CTRL_USE_STRAIGHT_BALANCE  0u
#define STRAIGHT_BALANCE_V_MIN    0.10f
#define STRAIGHT_BALANCE_W_THRESH 0.05f
#define STRAIGHT_BALANCE_KP       1.40f
#define STRAIGHT_BALANCE_KI       0.45f
#define STRAIGHT_BALANCE_I_LIM    0.10f
#define STRAIGHT_BALANCE_W_LIM    0.26f
#define STRAIGHT_TRIM_W           0.000f

/* ======== Side gain trim ======== */
/* 车体直行固定向左偏时设为正，固定向右偏时设为负。 */
#define STRAIGHT_SIDE_TRIM        0.06f

/* ======== Wheel loop anti-windup / low-speed ======== */
#define PI_AW_KAW                0.25f
#define LOW_SPEED_REF_RATIO      0.01f      /* relative to MAX_CPS */
#define U_STATIC_FF              0.06f

/* ======== Wheel startup assist (stiction breakaway) ======== */
#define START_ASSIST_ENABLE           1u
#define START_ASSIST_REF_RATIO      0.08f   /* |ref| above this enters assist logic */
#define START_ASSIST_MEAS_RATIO     0.02f   /* |meas| below this considered not moving */
#define START_ASSIST_REARM_RATIO    0.05f   /* |meas| above this rearms assist after wheel starts moving */
#define START_ASSIST_HOLD_MS          80u   /* how long to hold min duty once triggered */
#define START_ASSIST_MIN_U          0.14f   /* minimum duty used to break static friction */

/* ======== IMU yaw 阻尼 ======== */
/* 当 |w| < IMU_DAMP_ACTIVE_W 时启用阻尼 */
#define USE_IMU_DEFAULT          1u   /* 运行期仍可切换 */
#define IMU_DAMP_ACTIVE_W      0.20f
#define IMU_DAMP_K             0.008f /* w -= K * gz_dps */

/* ======== 指令仲裁 ======== */
/* 指令超时：超过这个时间未更新则自动刹停（ms） */
#define CMD_TIMEOUT_MS         300u
#define LINK_ACTIVE_TIMEOUT_MS 1200u
#define CMD_INPUT_DEADZONE    0.04f

/* ======== 日志/观测 ======== */
#define TELEMETRY_INTERVAL_MS  200u   /* 输出调参信息的最小间隔（ms） */
#define TELEMETRY_ENABLE         1u   /* 1=开启，0=关闭 */

/* ======== Link Protocol (binary frame) ======== */
#define LINK_PROTO_VER          0x01u
#define LINK_PAYLOAD_MAX         192u
#define LINK_RX_ACCUM_MAX        260u
#define ACK_TIMEOUT_MS            80u
#define ACK_RETRY_MAX              2u
#define FAULT_LOG_CAPACITY        64u
#define LINK_UART_TX_TIMEOUT_MS    8u
#define UART_ERR_STORM_THRESHOLD   4u
#define UART_ERR_STORM_WINDOW_MS  120u
#define UART_RECOVERY_COOLDOWN_MS 200u
