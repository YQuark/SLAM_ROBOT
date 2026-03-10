#pragma once

/* 车型 A：电池策略与闭环增益覆盖 */
#undef BATT_LIMIT_VOLTAGE
#define BATT_LIMIT_VOLTAGE 7.0f

#undef BATT_LIMIT_RECOVER_V
#define BATT_LIMIT_RECOVER_V 7.2f

#undef MAX_CPS
#define MAX_CPS 14500.0f

#undef OUTER_V_KP
#define OUTER_V_KP 0.65f

#undef ROBOT_CONFIG_VERSION
#define ROBOT_CONFIG_VERSION "vehicle_a-2026.03"
