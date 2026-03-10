#pragma once

/* ======== 周期调度（ms） ======== */
#define CTRL_PERIOD_MS          20u   /* 速度闭环控制周期 */
#define PS2_SCAN_INTERVAL_MS    50u
#define IMU_READ_INTERVAL_MS   100u
#define BATTERY_INTERVAL_MS    500u
#define OLED_INTERVAL_MS       500u
#define ESP_POLL_INTERVAL_MS    10u
#define PC_POLL_INTERVAL_MS     10u
#define HEARTBEAT_INTERVAL_MS  500u

/* ======== 平台功能开关 ======== */
#define PS2_ENABLE               0u
#define SELFTEST_ENABLE          0u
#define SELFTEST_SIGNAL_CHECK    1u
#define SELFTEST_ROTATE_MS    1000u

/* ======== 调试输出 ======== */
#define CTRL_DEBUG_ENABLE         0u
#define CTRL_DEBUG_INTERVAL_MS  200u
#define ESP_DEBUG_ENABLE          0u

/* ======== 日志/观测 ======== */
#define TELEMETRY_INTERVAL_MS  200u   /* 输出调参信息的最小间隔（ms） */
#define TELEMETRY_ENABLE         0u   /* 1=开启，0=关闭 */
