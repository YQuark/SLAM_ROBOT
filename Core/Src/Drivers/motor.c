#include "motor.h"
#include "robot_control.h"
#include "safety_manager.h"
#include <math.h>

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t ch; // TIM_CHANNEL_X
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    int8_t dir_polarity; // +1 keep, -1 invert
} _motor_def_t;

static _motor_def_t _M[4] = {
    {MOTOR_PWM_TIM, MOTOR_L1_PWM_CH, MOTOR_L1_DIR_PORT, MOTOR_L1_DIR_PIN, MOTOR_L1_DIR_POLARITY}, // L1
    {MOTOR_PWM_TIM, MOTOR_L2_PWM_CH, MOTOR_L2_DIR_PORT, MOTOR_L2_DIR_PIN, MOTOR_L2_DIR_POLARITY}, // L2
    {MOTOR_PWM_TIM, MOTOR_R1_PWM_CH, MOTOR_R1_DIR_PORT, MOTOR_R1_DIR_PIN, MOTOR_R1_DIR_POLARITY}, // R1
    {MOTOR_PWM_TIM, MOTOR_R2_PWM_CH, MOTOR_R2_DIR_PORT, MOTOR_R2_DIR_PIN, MOTOR_R2_DIR_POLARITY}, // R2
};

static float _duty_limit = MOTOR_DUTY_LIMIT_DEFAULT;

static inline uint32_t _arr(TIM_HandleTypeDef *htim) { return __HAL_TIM_GET_AUTORELOAD(htim); }
static inline void _set_ccr(TIM_HandleTypeDef *htim, uint32_t ch, uint32_t v) { __HAL_TIM_SET_COMPARE(htim, ch, v); }

void motor_init(void)
{
    for (int i = 0; i < 4; i++) {
        HAL_TIM_PWM_Start(_M[i].htim, _M[i].ch);
        _set_ccr(_M[i].htim, _M[i].ch, 0);
        HAL_GPIO_WritePin(_M[i].dir_port, _M[i].dir_pin, GPIO_PIN_RESET);
    }
}

void motor_deinit(void)
{
    for (int i = 0; i < 4; i++) {
        _set_ccr(_M[i].htim, _M[i].ch, 0);
        HAL_TIM_PWM_Stop(_M[i].htim, _M[i].ch);
    }
}

void motor_set(motor_id_t id, float speed)
{
    if (id < 0 || id > 3) {
        return;
    }
    _motor_def_t *m = &_M[id];

    if (!SafetyManager_AllowActuation()) {
        speed = 0.0f;
    }

    if (speed > 1.0f) {
        speed = 1.0f;
    }
    if (speed < -1.0f) {
        speed = -1.0f;
    }
    speed *= (float)m->dir_polarity;

    float d = fabsf(speed);
    if (d > _duty_limit) {
        d = _duty_limit;
    }

    // Zero command must coast: force both bridge inputs low.
    if (d < 1e-4f) {
        HAL_GPIO_WritePin(m->dir_port, m->dir_pin, GPIO_PIN_RESET);
        _set_ccr(m->htim, m->ch, 0);
        return;
    }

    // speed >= 0 -> DIR = 0, speed < 0 -> DIR = 1
    GPIO_PinState dir = (speed >= 0.0f) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(m->dir_port, m->dir_pin, dir);

    // With this 1-PWM + 1-DIR wiring, DIR=1 drive window is active-low.
    // Use (1-d) to keep forward/reverse magnitude symmetric.
    float pwm = (dir == GPIO_PIN_SET) ? (1.0f - d) : d;

    uint32_t arr = _arr(m->htim);
    uint32_t ccr = (uint32_t)lroundf(pwm * (float)arr);
    if (ccr > arr) {
        ccr = arr;
    }
    _set_ccr(m->htim, m->ch, ccr);
}

void motor_set_diff(float v, float w)
{
    float left = fmaxf(-1.0f, fminf(1.0f, v - w));
    float right = fmaxf(-1.0f, fminf(1.0f, v + w));

    motor_set(MOTOR_L1, left);
    motor_set(MOTOR_L2, left);
    motor_set(MOTOR_R1, right);
    motor_set(MOTOR_R2, right);

    RobotControl_SetCmd_ESP(v, w, HAL_GetTick());
}

void motor_coast_all(void)
{
    for (int i = 0; i < 4; i++) {
        HAL_GPIO_WritePin(_M[i].dir_port, _M[i].dir_pin, GPIO_PIN_RESET);
        _set_ccr(_M[i].htim, _M[i].ch, 0);
    }
}

void motor_brake_all(void)
{
    for (int i = 0; i < 4; i++) {
        HAL_GPIO_WritePin(_M[i].dir_port, _M[i].dir_pin, GPIO_PIN_SET);
        _set_ccr(_M[i].htim, _M[i].ch, _arr(_M[i].htim));
    }
}

void motor_set_polarity(motor_id_t id, int8_t polarity)
{
    if (id < 0 || id > 3) {
        return;
    }
    if (polarity == 0) {
        polarity = +1;
    }
    _M[id].dir_polarity = (polarity > 0) ? +1 : -1;
}

void motor_set_duty_limit(float limit)
{
    if (limit <= 0.0f) {
        limit = 0.05f;
    } else if (limit > 1.0f) {
        limit = 1.0f;
    }
    _duty_limit = limit;
}

float motor_get_duty_limit(void)
{
    return _duty_limit;
}

void motor_get_debug(uint16_t ccr[4], uint8_t dir[4])
{
    if (ccr) {
        ccr[0] = (uint16_t)__HAL_TIM_GET_COMPARE(_M[0].htim, _M[0].ch);
        ccr[1] = (uint16_t)__HAL_TIM_GET_COMPARE(_M[1].htim, _M[1].ch);
        ccr[2] = (uint16_t)__HAL_TIM_GET_COMPARE(_M[2].htim, _M[2].ch);
        ccr[3] = (uint16_t)__HAL_TIM_GET_COMPARE(_M[3].htim, _M[3].ch);
    }
    if (dir) {
        dir[0] = (uint8_t)HAL_GPIO_ReadPin(_M[0].dir_port, _M[0].dir_pin);
        dir[1] = (uint8_t)HAL_GPIO_ReadPin(_M[1].dir_port, _M[1].dir_pin);
        dir[2] = (uint8_t)HAL_GPIO_ReadPin(_M[2].dir_port, _M[2].dir_pin);
        dir[3] = (uint8_t)HAL_GPIO_ReadPin(_M[3].dir_port, _M[3].dir_pin);
    }
}
