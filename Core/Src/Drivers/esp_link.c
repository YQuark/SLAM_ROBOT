#include "esp_link.h"
#include "link_proto.h"
#include "robot_config.h"
#include "robot_control.h"

#define ESP_RX_FIFO_SIZE 256u

static UART_HandleTypeDef *s_huart = NULL;
static uint8_t s_rx_ch = 0u;
static uint8_t s_rx_fifo[ESP_RX_FIFO_SIZE];
static volatile uint16_t s_rx_fifo_head = 0u;
static volatile uint16_t s_rx_fifo_tail = 0u;
static volatile uint8_t s_rx_overflow = 0u;
static uint8_t s_uart_err_burst = 0u;
static uint32_t s_uart_err_window_ts = 0u;
static uint32_t s_last_recovery_ms = 0u;
static volatile uint8_t s_recover_pending = 0u;

static uint8_t esp_rx_fifo_push(uint8_t ch)
{
    uint16_t next = (uint16_t)((s_rx_fifo_head + 1u) % ESP_RX_FIFO_SIZE);
    if (next == s_rx_fifo_tail) {
        s_rx_overflow = 1u;
        return 0u;
    }
    s_rx_fifo[s_rx_fifo_head] = ch;
    s_rx_fifo_head = next;
    return 1u;
}

static uint8_t esp_rx_fifo_pop(uint8_t *out)
{
    uint8_t has_data = 0u;

    __disable_irq();
    if (s_rx_fifo_tail != s_rx_fifo_head) {
        *out = s_rx_fifo[s_rx_fifo_tail];
        s_rx_fifo_tail = (uint16_t)((s_rx_fifo_tail + 1u) % ESP_RX_FIFO_SIZE);
        has_data = 1u;
    }
    __enable_irq();

    return has_data;
}

static void esp_uart_restart_rx(void)
{
    if (!s_huart) {
        return;
    }
    (void)HAL_UART_AbortReceive(s_huart);
    (void)HAL_UART_Receive_IT(s_huart, &s_rx_ch, 1);
}

static void esp_uart_schedule_recovery(uint32_t now_ms)
{
    if ((s_last_recovery_ms != 0u) &&
        ((now_ms - s_last_recovery_ms) < UART_RECOVERY_COOLDOWN_MS)) {
        return;
    }
    s_recover_pending = 1u;
}

void ESP_Link_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_rx_fifo_head = 0u;
    s_rx_fifo_tail = 0u;
    s_rx_overflow = 0u;
    s_uart_err_burst = 0u;
    s_uart_err_window_ts = 0u;
    s_last_recovery_ms = 0u;
    s_recover_pending = 0u;
    LinkProto_Init(LINK_ID_ESP, huart);
    if (s_huart) {
        HAL_UART_Receive_IT(s_huart, &s_rx_ch, 1);
    }
}

void ESP_Link_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (!s_huart || huart != s_huart) return;
    (void)esp_rx_fifo_push(s_rx_ch);
    HAL_UART_Receive_IT(s_huart, &s_rx_ch, 1);
}

void ESP_Link_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t now_ms;

    if (!s_huart || huart != s_huart) return;

    now_ms = HAL_GetTick();

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    LinkProto_UartError(LINK_ID_ESP);
    RobotControl_ReportUartError(CMD_SRC_ESP);
    if ((s_uart_err_window_ts == 0u) ||
        ((now_ms - s_uart_err_window_ts) > UART_ERR_STORM_WINDOW_MS)) {
        s_uart_err_window_ts = now_ms;
        s_uart_err_burst = 1u;
    } else if (s_uart_err_burst < 0xFFu) {
        s_uart_err_burst++;
    }

    if (s_uart_err_burst >= UART_ERR_STORM_THRESHOLD) {
        esp_uart_schedule_recovery(now_ms);
    } else {
        esp_uart_restart_rx();
    }
}

void ESP_Link_Poll(const BatteryStatus_t *batt,
                   const MPU6050_Data_t *imu)
{
    uint8_t ch = 0u;
    uint32_t now_ms = HAL_GetTick();

    if (s_rx_overflow) {
        s_rx_overflow = 0u;
        LinkProto_UartError(LINK_ID_ESP);
        RobotControl_ReportUartError(CMD_SRC_ESP);
        esp_uart_schedule_recovery(now_ms);
    }

    if (s_recover_pending && s_huart) {
        s_recover_pending = 0u;
        (void)HAL_UART_DeInit(s_huart);
        (void)HAL_UART_Init(s_huart);
        s_last_recovery_ms = now_ms;
        s_uart_err_burst = 0u;
        s_uart_err_window_ts = 0u;
        RobotControl_ReportUartRecovery(CMD_SRC_ESP);
        esp_uart_restart_rx();
    }

    while (esp_rx_fifo_pop(&ch)) {
        LinkProto_RxByte(LINK_ID_ESP, ch);
    }

    LinkProto_Poll(LINK_ID_ESP, batt, imu);
}
