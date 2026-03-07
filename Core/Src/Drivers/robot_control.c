//
// Created by yushu on 2025/12/15.
//
#include "robot_control.h"
#include "robot_config.h"
#include "motor.h"
#include "encoder.h"
#include <math.h>
#include <string.h>
#include "stm32f4xx_hal.h"

/* ---------------- PI 控制器 ---------------- */
typedef struct {
    float kp;
    float ki;
    float integral;
    float out;
} PI_Controller_t;

/* ---------------- 全局变量 ---------------- */
static ControlMode_t s_mode = MODE_IDLE;
static uint32_t s_mode_switch_ts = 0;

static float s_open_v = 0.0f;
static float s_open_w = 0.0f;
static uint32_t s_open_ts = 0;

static RobotControlState_t s_st;
static PI_Controller_t s_pi[4];

static uint32_t s_ps2_ts = 0;
static uint32_t s_esp_ts = 0;
static uint32_t s_pc_ts = 0;
static float s_ps2_v = 0.0f, s_ps2_w = 0.0f;
static float s_esp_v = 0.0f, s_esp_w = 0.0f;
static float s_pc_v = 0.0f, s_pc_w = 0.0f;

static cmd_source_t s_prev_src = CMD_SRC_NONE;
static uint32_t s_src_switch_ts = 0;

static MPU6050_Data_t s_imu_last;
static float s_yaw_enc = 0.0f;
static float s_yaw_imu = 0.0f;
static float s_yaw_hold_ref = 0.0f;
static uint8_t s_yaw_hold_active = 0;

static float s_v_ref_filt = 0.0f;
static float s_w_ref_filt = 0.0f;
static float s_outer_i_v = 0.0f;
static float s_outer_i_w = 0.0f;
static float s_last_u[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static uint16_t s_start_assist_ticks[4] = {0u, 0u, 0u, 0u};

/* ---------------- 辅助函数 ---------------- */
static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static inline float signf_nonzero(float x)
{
    return (x >= 0.0f) ? 1.0f : -1.0f;
}

static float slew_limit(float current, float target, float rate_per_s, float dt)
{
    float max_step = rate_per_s * dt;
    float delta = target - current;
    if (delta > max_step) {
        delta = max_step;
    } else if (delta < -max_step) {
        delta = -max_step;
    }
    return current + delta;
}

static float PI_Update_AW(PI_Controller_t *pi, float err, float ff, float dt)
{
    float u_unsat = pi->kp * err + pi->integral + ff;
    float u_sat = clampf(u_unsat, -1.0f, 1.0f);

    if (CTRL_USE_AW) {
        pi->integral += pi->ki * err * dt + PI_AW_KAW * (u_sat - u_unsat) * dt;
    } else {
        pi->integral += pi->ki * err * dt;
    }
    pi->integral = clampf(pi->integral, -PI_INT_LIM, PI_INT_LIM);

    pi->out = u_sat;
    return u_sat;
}

static void reset_pi(void)
{
    s_pi[MOTOR_L1] = (PI_Controller_t){KP_L1, KI_L1, 0.0f, 0.0f};
    s_pi[MOTOR_L2] = (PI_Controller_t){KP_L2, KI_L2, 0.0f, 0.0f};
    s_pi[MOTOR_R1] = (PI_Controller_t){KP_R1, KI_R1, 0.0f, 0.0f};
    s_pi[MOTOR_R2] = (PI_Controller_t){KP_R2, KI_R2, 0.0f, 0.0f};
}

static void reset_runtime_states(void)
{
    s_v_ref_filt = 0.0f;
    s_w_ref_filt = 0.0f;
    s_outer_i_v = 0.0f;
    s_outer_i_w = 0.0f;
    s_yaw_enc = 0.0f;
    s_yaw_imu = 0.0f;
    s_yaw_hold_ref = 0.0f;
    s_yaw_hold_active = 0;
    for (int i = 0; i < 4; ++i) {
        s_last_u[i] = 0.0f;
        s_start_assist_ticks[i] = 0u;
    }
}

/* ---------------- 核心接口 ---------------- */
void RobotControl_Init(void)
{
    memset(&s_st, 0, sizeof(s_st));
    reset_pi();
    reset_runtime_states();
    s_mode = MODE_IDLE;
    s_mode_switch_ts = HAL_GetTick();
    s_prev_src = CMD_SRC_NONE;
    motor_set_duty_limit(0.8f);

    s_ps2_ts = 0;
    s_esp_ts = 0;
    s_pc_ts = 0;
    s_open_ts = 0;
    s_st.imu_enabled = USE_IMU_DEFAULT;
}

void RobotControl_SetMode(ControlMode_t mode)
{
    if (mode == s_mode) {
        return;
    }
    s_mode = mode;
    s_mode_switch_ts = HAL_GetTick();
    reset_pi();
    reset_runtime_states();
    motor_coast_all();
}

ControlMode_t RobotControl_GetMode(void)
{
    return s_mode;
}

void RobotControl_SetOpenLoopCmd(float v, float w)
{
    s_open_v = clampf(v, -1.0f, 1.0f);
    s_open_w = clampf(w, -1.0f, 1.0f);
    s_open_ts = HAL_GetTick();
}

void RobotControl_SetCmd_PS2(float v, float w, uint32_t now_ms)
{
    v = clampf(v, -1.0f, 1.0f);
    w = clampf(w, -1.0f, 1.0f);
    if (fabsf(v) < 0.06f) {
        v = 0.0f;
    }
    if (fabsf(w) < 0.06f) {
        w = 0.0f;
    }
    s_ps2_v = 0.75f * s_ps2_v + 0.25f * v;
    s_ps2_w = 0.75f * s_ps2_w + 0.25f * w;
    s_ps2_ts = now_ms;
}

void RobotControl_SetCmd_ESP(float v, float w, uint32_t now_ms)
{
    s_esp_v = clampf(v, -1.0f, 1.0f);
    s_esp_w = clampf(w, -1.0f, 1.0f);
    s_esp_ts = now_ms;
}

void RobotControl_SetCmd_PC(float v, float w, uint32_t now_ms)
{
    s_pc_v = clampf(v, -1.0f, 1.0f);
    s_pc_w = clampf(w, -1.0f, 1.0f);
    s_pc_ts = now_ms;
}

void RobotControl_UpdateIMU(const MPU6050_Data_t *imu, uint8_t ok, uint32_t now_ms)
{
    (void) now_ms;
    if (!s_st.imu_enabled) {
        s_st.imu_valid = 0;
        return;
    }
    if (ok && imu != NULL) {
        s_imu_last = *imu;
        s_st.imu_valid = 1;
        s_st.imu_err_cnt = 0;
    } else {
        if (++s_st.imu_err_cnt >= 5) {
            s_st.imu_enabled = 0;
            s_st.imu_valid = 0;
        }
    }
}

void RobotControl_SetIMUEnabled(uint8_t en)
{
    s_st.imu_enabled = en;
    if (!en) {
        s_st.imu_valid = 0;
    }
}

static void choose_cmd(uint32_t now_ms, float *v, float *w, cmd_source_t *src)
{
    *v = 0.0f;
    *w = 0.0f;
    *src = CMD_SRC_NONE;
    uint8_t ps2_ok = (s_ps2_ts != 0u) && ((now_ms - s_ps2_ts) <= CMD_TIMEOUT_MS);
    uint8_t pc_ok = (s_pc_ts != 0u) && ((now_ms - s_pc_ts) <= CMD_TIMEOUT_MS);
    uint8_t esp_ok = (s_esp_ts != 0u) && ((now_ms - s_esp_ts) <= CMD_TIMEOUT_MS);
    uint32_t best_ts = 0u;

    /* 超时窗口内最新命令优先，避免固定优先级抢占 */
    if (ps2_ok && s_ps2_ts >= best_ts) {
        *v = s_ps2_v;
        *w = s_ps2_w;
        *src = CMD_SRC_PS2;
        best_ts = s_ps2_ts;
    }
    if (pc_ok && s_pc_ts >= best_ts) {
        *v = s_pc_v;
        *w = s_pc_w;
        *src = CMD_SRC_PC;
        best_ts = s_pc_ts;
    }
    if (esp_ok && s_esp_ts >= best_ts) {
        *v = s_esp_v;
        *w = s_esp_w;
        *src = CMD_SRC_ESP;
    }
}

static void update_observer(float dt)
{
    float meas_l = 0.5f * (g_encoders[ENC_L1].vel_cps + g_encoders[ENC_L2].vel_cps);
    float meas_r = 0.5f * (g_encoders[ENC_R1].vel_cps + g_encoders[ENC_R2].vel_cps);
    float v_est = 0.5f * (meas_l + meas_r) / MAX_CPS;
    float w_est = 0.5f * (meas_r - meas_l) / MAX_CPS;

    s_st.v_est = v_est;
    s_st.w_est = w_est;

    s_yaw_enc += w_est * dt;
    if (s_st.imu_enabled && s_st.imu_valid) {
        s_yaw_imu += (s_imu_last.gz_dps / IMU_W_DPS_SCALE) * dt;
    }
    s_st.yaw_est = YAW_FUSION_ALPHA * s_yaw_enc + (1.0f - YAW_FUSION_ALPHA) * s_yaw_imu;
}

static void stop_outputs_and_observe(void)
{
    motor_coast_all();
    for (int i = 0; i < 4; ++i) {
        s_st.ref_cps[i] = 0.0f;
        s_st.meas_cps[i] = g_encoders[i].vel_cps;
        s_st.u_out[i] = 0.0f;
        s_last_u[i] = 0.0f;
    }
}

void RobotControl_Update(float dt, uint32_t now_ms)
{
    float v_cmd = 0.0f;
    float w_cmd = 0.0f;
    cmd_source_t src = CMD_SRC_NONE;
    float ref_cps[4];
    float u_cmd[4];
    const uint16_t assist_hold_ticks =
        (uint16_t)((START_ASSIST_HOLD_MS + CTRL_PERIOD_MS - 1u) / CTRL_PERIOD_MS);

    choose_cmd(now_ms, &v_cmd, &w_cmd, &src);

    if (s_mode == MODE_OPEN_LOOP) {
        if ((s_open_ts != 0u) &&
            ((now_ms - s_open_ts) <= CMD_TIMEOUT_MS) &&
            ((s_open_v != 0.0f) || (s_open_w != 0.0f))) {
            v_cmd = s_open_v;
            w_cmd = s_open_w;
            src = CMD_SRC_ESP;
        }
    }

    if (src != s_prev_src) {
        s_prev_src = src;
        s_src_switch_ts = now_ms;
    }
    s_st.src_transition_active = 0u;

    s_st.v_cmd = v_cmd;
    s_st.w_cmd = w_cmd;
    s_st.src = src;

    update_observer(dt);

    s_st.mode_transition_active = 0u;

    if (s_mode == MODE_IDLE) {
        stop_outputs_and_observe();
        return;
    }

    {
        float v_rate = V_SLEW_MAX;
        float w_rate = W_SLEW_MAX;
        /* 目标为 0 时立即归零，不经过 slew */
        if (fabsf(v_cmd) < 0.01f) {
            s_v_ref_filt = 0.0f;
        } else {
            s_v_ref_filt = slew_limit(s_v_ref_filt, v_cmd, v_rate, dt);
        }
        if (fabsf(w_cmd) < 0.01f) {
            s_w_ref_filt = 0.0f;
        } else {
            s_w_ref_filt = slew_limit(s_w_ref_filt, w_cmd, w_rate, dt);
        }
    }


    s_st.v_ref_filt = s_v_ref_filt;
    s_st.w_ref_filt = s_w_ref_filt;

    if (s_mode == MODE_OPEN_LOOP) {
        float vL = clampf(s_v_ref_filt - s_w_ref_filt, -1.0f, 1.0f);
        float vR = clampf(s_v_ref_filt + s_w_ref_filt, -1.0f, 1.0f);
        motor_set(MOTOR_L1, vL);
        motor_set(MOTOR_L2, vL);
        motor_set(MOTOR_R1, vR);
        motor_set(MOTOR_R2, vR);

        s_st.ref_cps[ENC_L1] = vL * MAX_CPS;
        s_st.ref_cps[ENC_L2] = vL * MAX_CPS;
        s_st.ref_cps[ENC_R1] = vR * MAX_CPS;
        s_st.ref_cps[ENC_R2] = vR * MAX_CPS;
        for (int i = 0; i < 4; ++i) {
            s_st.meas_cps[i] = g_encoders[i].vel_cps;
        }
        s_st.u_out[ENC_L1] = vL;
        s_st.u_out[ENC_L2] = vL;
        s_st.u_out[ENC_R1] = vR;
        s_st.u_out[ENC_R2] = vR;
        return;
    }

    {
        float v_ref = s_v_ref_filt;
        float w_ref = s_w_ref_filt;

        /* Simplified: keep only v/w -> wheel speed mapping. */
        s_outer_i_v = 0.0f;
        s_outer_i_w = 0.0f;
        s_yaw_hold_active = 0u;

        ref_cps[ENC_L1] = clampf(v_ref - w_ref, -1.0f, 1.0f) * MAX_CPS;
        ref_cps[ENC_L2] = ref_cps[ENC_L1];
        ref_cps[ENC_R1] = clampf(v_ref + w_ref, -1.0f, 1.0f) * MAX_CPS;
        ref_cps[ENC_R2] = ref_cps[ENC_R1];
    }

    for (int i = 0; i < 4; ++i) {
        float meas = g_encoders[i].vel_cps;
        float ref = ref_cps[i];
        float ff = 0.0f;
        float u = 0.0f;

        s_st.ref_cps[i] = ref;
        s_st.meas_cps[i] = meas;

        /* 低速或停止时清零积分与输出，避免抖动 */
        if (fabsf(ref) < (LOW_SPEED_REF_RATIO * MAX_CPS)) {
            s_pi[i].integral = 0.0f;
            s_last_u[i] = 0.0f;
            s_start_assist_ticks[i] = 0u;
            u = 0.0f;
        } else if (fabsf(meas) > (1.5f * MAX_CPS)) {
            /* 异常速度时强制清零 */
            reset_pi();
            u = 0.0f;
            s_last_u[i] = 0.0f;
            s_start_assist_ticks[i] = 0u;
        } else {
            if (CTRL_USE_STATIC_FF && fabsf(ref) > 1.0f) {
                ff = U_STATIC_FF * signf_nonzero(ref);
            }
            u = PI_Update_AW(&s_pi[i], ref - meas, ff, dt);
            u = slew_limit(s_last_u[i], u, U_SLEW_MAX, dt);
            s_last_u[i] = u;

#if START_ASSIST_ENABLE
            if (fabsf(ref) > (START_ASSIST_REF_RATIO * MAX_CPS) &&
                fabsf(meas) < (START_ASSIST_MEAS_RATIO * MAX_CPS)) {
                s_start_assist_ticks[i] = assist_hold_ticks;
            }
            if (s_start_assist_ticks[i] > 0u) {
                float assist_u = START_ASSIST_MIN_U * signf_nonzero(ref);
                if (fabsf(u) < START_ASSIST_MIN_U) {
                    u = assist_u;
                    s_pi[i].integral = 0.0f;
                    s_last_u[i] = u;
                }
                s_start_assist_ticks[i]--;
            }
#endif
        }

        u_cmd[i] = u;
    }

    motor_set(MOTOR_L1, u_cmd[ENC_L1]);
    motor_set(MOTOR_L2, u_cmd[ENC_L2]);
    motor_set(MOTOR_R1, u_cmd[ENC_R1]);
    motor_set(MOTOR_R2, u_cmd[ENC_R2]);
    for (int i = 0; i < 4; ++i) {
        s_st.u_out[i] = u_cmd[i];
    }
}

const RobotControlState_t *RobotControl_GetState(void)
{
    return &s_st;
}

