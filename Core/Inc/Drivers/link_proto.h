#ifndef LINK_PROTO_H
#define LINK_PROTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "battery.h"
#include "mpu6050.h"
#include "link_diag.h"

enum {
    LINK_MSG_CMD_PING = 0x01,
    LINK_MSG_CMD_GET_STATUS = 0x02,
    LINK_MSG_CMD_SET_DRIVE = 0x10,
    LINK_MSG_CMD_SET_MODE = 0x11,
    LINK_MSG_CMD_SET_IMU = 0x12,
    LINK_MSG_CMD_GET_DIAG = 0x30,
    LINK_MSG_CMD_CLEAR_DIAG = 0x31,
    LINK_MSG_CMD_GET_FAULT_LOG = 0x32,
    LINK_MSG_CMD_CLEAR_FAULT_LOG = 0x33,
    LINK_MSG_CMD_GET_EVENT_LOG = 0x34,
    LINK_MSG_CMD_GET_CTRL_TRACE = 0x35,
    LINK_MSG_ACK = 0x80,
    LINK_MSG_NACK = 0x81,
};

enum {
    LINK_FLAG_ACK_REQ = 0x01,
    LINK_FLAG_IS_ACK = 0x02,
    LINK_FLAG_IS_NACK = 0x04,
};

enum {
    LINK_ERR_OK = 0x0000,

    LINK_ERR_PROTO_SOF = 0x0101,
    LINK_ERR_PROTO_EOF = 0x0102,
    LINK_ERR_PROTO_LEN = 0x0103,
    LINK_ERR_PROTO_CRC = 0x0104,
    LINK_ERR_PROTO_COBS = 0x0105,
    LINK_ERR_PROTO_SEQ = 0x0106,

    LINK_ERR_PARSE_UNKNOWN_CMD = 0x0201,
    LINK_ERR_PARSE_PAYLOAD_LEN = 0x0202,
    LINK_ERR_PARSE_PARAM_RANGE = 0x0203,

    LINK_ERR_CTRL_MODE_REJECT = 0x0301,

    LINK_ERR_UART_IO = 0x0401,

    LINK_ERR_SAFE_UV_LIMIT = 0x0501,
    LINK_ERR_SAFE_UV_CUTOFF = 0x0502,

    LINK_ERR_RES_RX_OVERFLOW = 0x0601,
    LINK_ERR_RES_LOG_OVERFLOW = 0x0602,
};

void LinkProto_Init(link_id_t link_id, UART_HandleTypeDef *huart);
void LinkProto_RxByte(link_id_t link_id, uint8_t byte);
void LinkProto_UartError(link_id_t link_id);
void LinkProto_Poll(link_id_t link_id,
                    const BatteryStatus_t *batt,
                    const MPU6050_Data_t *imu);

#ifdef __cplusplus
}
#endif

#endif
