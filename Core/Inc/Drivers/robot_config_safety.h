#pragma once

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

/* ======== 指令仲裁 ======== */
/* 指令超时：超过这个时间未更新则自动刹停（ms） */
#define CMD_TIMEOUT_MS         300u
