#include "safety_manager.h"
#include <string.h>

typedef struct {
    SafetyState_t from;
    SafetyState_t to;
    uint8_t allow;
} SafetyTransition_t;

static const SafetyTransition_t k_transition_table[] = {
    {SAFETY_STATE_RUNNING, SAFETY_STATE_RUNNING, 1u},
    {SAFETY_STATE_RUNNING, SAFETY_STATE_DEGRADED, 1u},
    {SAFETY_STATE_RUNNING, SAFETY_STATE_SAFE_STOP, 0u},
    {SAFETY_STATE_RUNNING, SAFETY_STATE_FAULT_LATCHED, 0u},

    {SAFETY_STATE_DEGRADED, SAFETY_STATE_RUNNING, 1u},
    {SAFETY_STATE_DEGRADED, SAFETY_STATE_DEGRADED, 1u},
    {SAFETY_STATE_DEGRADED, SAFETY_STATE_SAFE_STOP, 0u},
    {SAFETY_STATE_DEGRADED, SAFETY_STATE_FAULT_LATCHED, 0u},

    {SAFETY_STATE_SAFE_STOP, SAFETY_STATE_RUNNING, 1u},
    {SAFETY_STATE_SAFE_STOP, SAFETY_STATE_DEGRADED, 1u},
    {SAFETY_STATE_SAFE_STOP, SAFETY_STATE_SAFE_STOP, 0u},
    {SAFETY_STATE_SAFE_STOP, SAFETY_STATE_FAULT_LATCHED, 0u},

    {SAFETY_STATE_FAULT_LATCHED, SAFETY_STATE_FAULT_LATCHED, 0u},
};

enum {
    SAFETY_EVENT_BUF_SIZE = 16,
};

static SafetyState_t s_state = SAFETY_STATE_RUNNING;
static uint8_t s_allow = 1u;
static SafetyInputs_t s_last_inputs;

static SafetyEvent_t s_events[SAFETY_EVENT_BUF_SIZE];
static uint8_t s_evt_head = 0u;
static uint8_t s_evt_tail = 0u;

static uint32_t inputs_to_mask(const SafetyInputs_t *inputs)
{
    uint32_t m = SAFETY_SRC_NONE;
    if (inputs->battery_uv_limit) {
        m |= SAFETY_SRC_BATTERY_UV_LIMIT;
    }
    if (inputs->battery_uv_cutoff) {
        m |= SAFETY_SRC_BATTERY_UV_CUTOFF;
    }
    if (inputs->link_timeout) {
        m |= SAFETY_SRC_LINK_TIMEOUT;
    }
    if (inputs->imu_cont_fail) {
        m |= SAFETY_SRC_IMU_CONT_FAIL;
    }
    if (inputs->ctrl_period_abnormal) {
        m |= SAFETY_SRC_CTRL_PERIOD_ABNORMAL;
    }
    if (inputs->hard_fault) {
        m |= SAFETY_SRC_HARD_FAULT;
    }
    return m;
}

static SafetyState_t compute_target_state(const SafetyInputs_t *inputs)
{
    if (inputs->hard_fault) {
        return SAFETY_STATE_FAULT_LATCHED;
    }
    if (inputs->battery_uv_cutoff || inputs->ctrl_period_abnormal) {
        return SAFETY_STATE_SAFE_STOP;
    }
    if (inputs->battery_uv_limit || inputs->link_timeout || inputs->imu_cont_fail) {
        return SAFETY_STATE_DEGRADED;
    }
    return SAFETY_STATE_RUNNING;
}

static uint8_t lookup_allow(SafetyState_t from, SafetyState_t to)
{
    for (uint32_t i = 0; i < (uint32_t)(sizeof(k_transition_table) / sizeof(k_transition_table[0])); ++i) {
        if (k_transition_table[i].from == from && k_transition_table[i].to == to) {
            return k_transition_table[i].allow;
        }
    }
    return 0u;
}

static void push_event(uint32_t now_ms, uint32_t src_mask, SafetyState_t old_state, SafetyState_t new_state, uint16_t extra)
{
    uint8_t next = (uint8_t)((s_evt_head + 1u) % SAFETY_EVENT_BUF_SIZE);
    if (next == s_evt_tail) {
        s_evt_tail = (uint8_t)((s_evt_tail + 1u) % SAFETY_EVENT_BUF_SIZE);
    }
    s_events[s_evt_head].timestamp_ms = now_ms;
    s_events[s_evt_head].source_mask = src_mask;
    s_events[s_evt_head].old_state = old_state;
    s_events[s_evt_head].new_state = new_state;
    s_events[s_evt_head].extra_code = extra;
    s_evt_head = next;
}

void SafetyManager_Init(void)
{
    memset(&s_last_inputs, 0, sizeof(s_last_inputs));
    s_state = SAFETY_STATE_RUNNING;
    s_allow = 1u;
    s_evt_head = 0u;
    s_evt_tail = 0u;
}

SafetyState_t SafetyManager_Update(const SafetyInputs_t *inputs, uint32_t now_ms)
{
    if (inputs == NULL) {
        return s_state;
    }
    s_last_inputs = *inputs;

    SafetyState_t target = compute_target_state(inputs);
    if (s_state == SAFETY_STATE_FAULT_LATCHED) {
        s_allow = 0u;
        return s_state;
    }

    if (target != s_state) {
        SafetyState_t old = s_state;
        s_state = target;
        s_allow = lookup_allow(old, target);
        push_event(now_ms, inputs_to_mask(inputs), old, target, inputs->extra_code);
    } else {
        s_allow = lookup_allow(s_state, s_state);
    }

    return s_state;
}

SafetyState_t SafetyManager_GetState(void)
{
    return s_state;
}

uint8_t SafetyManager_AllowActuation(void)
{
    return s_allow;
}

void SafetyManager_ClearFaultLatch(void)
{
    if (s_state != SAFETY_STATE_FAULT_LATCHED) {
        return;
    }
    s_state = compute_target_state(&s_last_inputs);
    s_allow = lookup_allow(SAFETY_STATE_RUNNING, s_state);
}

uint8_t SafetyManager_PopEvent(SafetyEvent_t *out_event)
{
    if (s_evt_tail == s_evt_head || out_event == NULL) {
        return 0u;
    }
    *out_event = s_events[s_evt_tail];
    s_evt_tail = (uint8_t)((s_evt_tail + 1u) % SAFETY_EVENT_BUF_SIZE);
    return 1u;
}

uint8_t SafetyManager_GetLatestEvent(SafetyEvent_t *out_event)
{
    if (s_evt_head == s_evt_tail || out_event == NULL) {
        return 0u;
    }
    uint8_t idx = (uint8_t)((s_evt_head + SAFETY_EVENT_BUF_SIZE - 1u) % SAFETY_EVENT_BUF_SIZE);
    *out_event = s_events[idx];
    return 1u;
}

const char *SafetyManager_StateName(SafetyState_t state)
{
    switch (state) {
        case SAFETY_STATE_RUNNING:
            return "RUNNING";
        case SAFETY_STATE_DEGRADED:
            return "DEGRADED";
        case SAFETY_STATE_SAFE_STOP:
            return "SAFE_STOP";
        case SAFETY_STATE_FAULT_LATCHED:
            return "FAULT_LATCHED";
        default:
            return "UNKNOWN";
    }
}
