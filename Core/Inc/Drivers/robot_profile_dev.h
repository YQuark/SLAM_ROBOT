#pragma once

/* 开发调试 profile：提高可观测性、收紧超时 */
#undef TELEMETRY_ENABLE
#define TELEMETRY_ENABLE 1u

#undef CTRL_DEBUG_ENABLE
#define CTRL_DEBUG_ENABLE 1u

#undef CMD_TIMEOUT_MS
#define CMD_TIMEOUT_MS 250u

#undef ROBOT_CONFIG_VERSION
#define ROBOT_CONFIG_VERSION "dev-2026.03"
