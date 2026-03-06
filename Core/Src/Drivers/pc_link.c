#include "pc_link.h"
#include "link_proto.h"

static UART_HandleTypeDef *s_huart = NULL;
static uint8_t s_rx_ch = 0u;

void PC_Link_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    LinkProto_Init(LINK_ID_PC, huart);
    if (s_huart) {
        HAL_UART_Receive_IT(s_huart, &s_rx_ch, 1);
    }
}

void PC_Link_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (!s_huart || huart != s_huart) return;
    LinkProto_RxByte(LINK_ID_PC, s_rx_ch);
    HAL_UART_Receive_IT(s_huart, &s_rx_ch, 1);
}

void PC_Link_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (!s_huart || huart != s_huart) return;

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    LinkProto_UartError(LINK_ID_PC);
    HAL_UART_Receive_IT(s_huart, &s_rx_ch, 1);
}

void PC_Link_Poll(const BatteryStatus_t *batt,
                  const MPU6050_Data_t *imu)
{
    LinkProto_Poll(LINK_ID_PC, batt, imu);
}
