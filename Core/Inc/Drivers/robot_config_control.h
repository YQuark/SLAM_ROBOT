#pragma once

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

#define KP_L1                0.00025f
#define KI_L1                0.00040f
#define KP_L2                0.00025f
#define KI_L2                0.00040f
#define KP_R1                0.00025f
#define KI_R1                0.00040f
#define KP_R2                0.00025f
#define KI_R2                0.00040f

/* ======== Control optimization toggles ======== */
#define CTRL_USE_DUAL_LOOP           0u
#define CTRL_USE_YAW_HOLD            0u
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
#define YAW_FUSION_ALPHA         0.70f      /* 0..1, higher=encoder dominated */
#define IMU_W_DPS_SCALE         180.0f      /* dps that maps to w=1.0 */
#define YAW_HOLD_KP              1.10f
#define YAW_HOLD_W_THRESH        0.06f

/* ======== Wheel loop anti-windup / low-speed ======== */
#define PI_AW_KAW                0.25f
#define LOW_SPEED_REF_RATIO      0.01f      /* relative to MAX_CPS */
#define U_STATIC_FF              0.06f

/* ======== Wheel startup assist (stiction breakaway) ======== */
#define START_ASSIST_ENABLE           1u
#define START_ASSIST_REF_RATIO      0.08f   /* |ref| above this enters assist logic */
#define START_ASSIST_MEAS_RATIO     0.02f   /* |meas| below this considered not moving */
#define START_ASSIST_HOLD_MS          80u   /* how long to hold min duty once triggered */
#define START_ASSIST_MIN_U          0.14f   /* minimum duty used to break static friction */

/* ======== IMU yaw 阻尼 ======== */
/* 当 |w| < IMU_DAMP_ACTIVE_W 时启用阻尼 */
#define USE_IMU_DEFAULT          0u   /* 运行期仍可切换 */
#define IMU_DAMP_ACTIVE_W      0.20f
#define IMU_DAMP_K             0.008f /* w -= K * gz_dps */
