#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SAFETY_STATE_RUNNING = 0,
    SAFETY_STATE_DEGRADED,
    SAFETY_STATE_SAFE_STOP,
    SAFETY_STATE_FAULT_LATCHED,
    SAFETY_STATE_COUNT
} SafetyState_t;

typedef enum {
    SAFETY_SRC_NONE = 0,
    SAFETY_SRC_BATTERY_UV_LIMIT = 1u << 0,
    SAFETY_SRC_BATTERY_UV_CUTOFF = 1u << 1,
    SAFETY_SRC_LINK_TIMEOUT = 1u << 2,
    SAFETY_SRC_IMU_CONT_FAIL = 1u << 3,
    SAFETY_SRC_CTRL_PERIOD_ABNORMAL = 1u << 4,
    SAFETY_SRC_HARD_FAULT = 1u << 5,
} SafetySource_t;

typedef struct {
    uint8_t battery_uv_limit;
    uint8_t battery_uv_cutoff;
    uint8_t link_timeout;
    uint8_t imu_cont_fail;
    uint8_t ctrl_period_abnormal;
    uint8_t hard_fault;
    uint16_t extra_code;
} SafetyInputs_t;

typedef struct {
    uint32_t timestamp_ms;
    uint32_t source_mask;
    SafetyState_t old_state;
    SafetyState_t new_state;
    uint16_t extra_code;
} SafetyEvent_t;

void SafetyManager_Init(void);
SafetyState_t SafetyManager_Update(const SafetyInputs_t *inputs, uint32_t now_ms);
SafetyState_t SafetyManager_GetState(void);
uint8_t SafetyManager_AllowActuation(void);
void SafetyManager_ClearFaultLatch(void);

uint8_t SafetyManager_PopEvent(SafetyEvent_t *out_event);
uint8_t SafetyManager_GetLatestEvent(SafetyEvent_t *out_event);

const char *SafetyManager_StateName(SafetyState_t state);

#ifdef __cplusplus
}
#endif

#endif
