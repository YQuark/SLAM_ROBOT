# Serial Protocols

## Scope

This document describes the serial interfaces implemented in the firmware under `Core/Src/main.c`, `Core/Src/Drivers/pc_link.c`, `Core/Src/Drivers/esp_link.c`, and `Core/Src/Drivers/link_proto.c`.

Unless otherwise noted, all UART ports use:

- Baud rate: `115200`
- Data bits: `8`
- Parity: `None`
- Stop bits: `1`
- Flow control: `None`

Source of truth:

- UART config: `Core/Src/usart.c`
- Binary protocol: `Core/Src/Drivers/link_proto.c`, `Core/Inc/Drivers/link_proto.h`
- PC ASCII console: `Core/Src/Drivers/pc_link.c`
- ESP bridge peer: `ESP01S/ESP01S/ESP01S.ino`

Project boundary:

- This repository contains STM32 firmware and its on-board serial protocols only.
- ROS, SLAM, navigation, and host-side bridge programs belong to a separate project and are not part of this repository.

## Port Map

### USART1

- Purpose: local debug log and telemetry output
- Pins:
  - `PB6` -> `USART1_TX`
  - `PB7` -> `USART1_RX`
- Typical traffic:
  - boot log
  - MPU6050 status
  - IMU calibration messages
  - optional telemetry CSV for VOFA+ FireWater

### USART2

- Purpose: ESP-01S bridge link
- Pins:
  - `PD5` -> `USART2_TX`
  - `PA3` -> `USART2_RX`
- Firmware module: `ESP_Link_*`
- Protocol: binary framed protocol only

### USART3

- Purpose: PC host link
- Pins:
  - `PB10` -> `USART3_TX`
  - `PB11` -> `USART3_RX`
- Firmware module: `PC_Link_*`
- Protocols:
  - binary framed protocol
  - ASCII command console

## Polling And Timing

From `Core/Inc/Drivers/robot_config.h`:

- `ESP_POLL_INTERVAL_MS = 10`
- `PC_POLL_INTERVAL_MS = 10`
- `IMU_READ_INTERVAL_MS = 100`
- `TELEMETRY_INTERVAL_MS = 200`
- `CMD_TIMEOUT_MS = 300`
- `ACK_TIMEOUT_MS = 80`
- `ACK_RETRY_MAX = 2`
- `FAULT_LOG_CAPACITY = 64`

## Binary Framed Protocol

### Supported Links

- `LINK_ID_ESP = 0`
- `LINK_ID_PC = 1`

The same frame format is used on `USART2` and `USART3`.

### Outer Framing

- Transport encoding: `COBS`
- Frame delimiter after COBS payload: single trailing `0x00`

### Decoded Frame Layout

Decoded payload before COBS encoding:

1. `SOF` `1B` = `0xA5`
2. `VER` `1B` = `0x01`
3. `MSG_TYPE` `1B`
4. `FLAGS` `1B`
5. `SEQ` `1B`
6. `PAYLOAD_LEN` `2B`, little-endian
7. `PAYLOAD` `N B`
8. `CRC16` `2B`, little-endian
9. `EOF` `1B` = `0x5A`

CRC details:

- Algorithm: `CRC-16/CCITT-FALSE`
- Init: `0xFFFF`
- Poly: `0x1021`
- Covered bytes: `VER .. payload`, length `6 + payload_len`

Limits:

- `LINK_PAYLOAD_MAX = 192`
- RX accumulation buffer: `260`
- encoded buffer: `300`

### Flags

- `0x01` `LINK_FLAG_ACK_REQ`: request ACK/NACK
- `0x02` `LINK_FLAG_IS_ACK`: ACK frame
- `0x04` `LINK_FLAG_IS_NACK`: NACK frame

### Sequence Handling

- Duplicate suppression is enabled for all commands except `SET_DRIVE`.
- If a duplicate request with same `seq` and `msg_type` arrives and a cached response exists, the previous encoded response is resent.

### Message Types

- `0x01` `PING`
- `0x02` `GET_STATUS`
- `0x10` `SET_DRIVE`
- `0x11` `SET_MODE`
- `0x12` `SET_IMU`
- `0x30` `GET_DIAG`
- `0x31` `CLEAR_DIAG`
- `0x32` `GET_FAULT_LOG`
- `0x33` `CLEAR_FAULT_LOG`
- `0x80` `ACK`
- `0x81` `NACK`

## Binary Command Payloads

### `PING` `0x01`

- Request payload: empty
- ACK extra payload:
  - `0xAA`

### `GET_STATUS` `0x02`

- Request payload: empty
- ACK extra payload layout:
  1. `tick_ms` `u32`
  2. `mode` `u8`
  3. `src` `u8`
  4. `battery_mv` `u16`
  5. `battery_pct_x10` `u16`
  6. `v_cmd_q15` `i16`
  7. `w_cmd_q15` `i16`
  8. `gz_dps_x10` `i16`
  9. `imu_enabled` `u8`
  10. `imu_valid` `u8`
  11. `enc_vel_cps[4]` `i16 x 4`
  12. `v_est_q15` `i16`
  13. `w_est_q15` `i16`
  14. `enc_fault_mask` `u8`

Notes:

- `v_cmd_q15` and `w_cmd_q15` are normalized command values scaled by `32767`.
- `enc_vel_cps` order follows `ENC_L1`, `ENC_L2`, `ENC_R1`, `ENC_R2`.
- `v_est_q15` and `w_est_q15` are firmware-side estimated chassis linear/angular velocity, also scaled by `32767`.
- `enc_fault_mask` exposes encoder fault state bits for host-side diagnostics.

### `SET_DRIVE` `0x10`

- Request payload:
  1. `v_q15` `i16`
  2. `w_q15` `i16`

Behavior:

- `LINK_ID_ESP` updates `RobotControl_SetCmd_ESP`
- `LINK_ID_PC` updates `RobotControl_SetCmd_PC`

### `SET_MODE` `0x11`

- Request payload:
  1. `mode` `u8`

Valid values:

- `0` `MODE_IDLE`
- `1` `MODE_OPEN_LOOP`
- `2` `MODE_CLOSED_LOOP`

### `SET_IMU` `0x12`

- Request payload:
  1. `enable` `u8`

Behavior:

- zero means disable
- non-zero means enable

### `GET_DIAG` `0x30`

- Request payload: empty
- ACK extra payload layout:
  1. `rx_total` `u32`
  2. `rx_ok` `u32`
  3. `rx_drop` `u32`
  4. `cobs_err` `u32`
  5. `crc_err` `u32`
  6. `len_err` `u32`
  7. `sof_err` `u32`
  8. `eof_err` `u32`
  9. `seq_dup` `u32`
  10. `uart_err` `u32`
  11. `tx_fail` `u32`
  12. `last_err` `u16`

### `CLEAR_DIAG` `0x31`

- Request payload: empty
- Effect: clears snapshot counters for the current link only

### `GET_FAULT_LOG` `0x32`

- Request payload:
  - byte 0 optional: `cursor`
  - byte 1 optional: `max_records`

Defaults:

- `cursor = 0`
- `max_records = 4`
- firmware clamps `max_records` to `1..8`

ACK extra payload:

1. `next_cursor` `u8`
2. `count` `u8`
3. `dropped_fault_count` `u16`
4. repeated records:
   - `ts_ms` `u32`
   - `link_id` `u8`
   - `severity` `u8`
   - `err_code` `u16`
   - `seq` `u8`
   - `msg_type` `u8`
   - `aux0` `u16`
   - `aux1` `u16`

### `CLEAR_FAULT_LOG` `0x33`

- Request payload: empty
- Effect: clears global fault ring buffer

## ACK/NACK Payload

Both ACK and NACK payloads begin with:

1. `seq` `u8`
2. `err_code` `u16`
3. `detail` `u8`

For ACK, command-specific response data may follow.

## Binary Error Codes

- `0x0000` `LINK_ERR_OK`
- `0x0101` `LINK_ERR_PROTO_SOF`
- `0x0102` `LINK_ERR_PROTO_EOF`
- `0x0103` `LINK_ERR_PROTO_LEN`
- `0x0104` `LINK_ERR_PROTO_CRC`
- `0x0105` `LINK_ERR_PROTO_COBS`
- `0x0106` `LINK_ERR_PROTO_SEQ`
- `0x0201` `LINK_ERR_PARSE_UNKNOWN_CMD`
- `0x0202` `LINK_ERR_PARSE_PAYLOAD_LEN`
- `0x0203` `LINK_ERR_PARSE_PARAM_RANGE`
- `0x0301` `LINK_ERR_CTRL_MODE_REJECT`
- `0x0401` `LINK_ERR_UART_IO`
- `0x0501` `LINK_ERR_SAFE_UV_LIMIT`
- `0x0502` `LINK_ERR_SAFE_UV_CUTOFF`
- `0x0601` `LINK_ERR_RES_RX_OVERFLOW`
- `0x0602` `LINK_ERR_RES_LOG_OVERFLOW`

## USART3 ASCII Console

ASCII parsing is implemented in `Core/Src/Drivers/pc_link.c`.

Rules:

- line oriented
- accepts printable ASCII and tab
- line end: `CR` or `LF`
- binary parser runs first, ASCII parsing is best-effort

### Commands

#### `PING`

Response:

```text
OK PONG
```

#### `HELP`

Response:

```text
OK CMDS: PING, HELP, STATUS, MODE <0|1|2>, DRIVE <v> <w>, CTRL <v> <w>, STOP, IMU <0|1>, STREAM <ON|OFF> [hz], RATE <hz>
```

#### `STATUS`

Response format:

```text
OK STATUS mode=<u> src=<u> v=<f> w=<f> vbat=<f> imu_en=<u> imu_ok=<u> gz=<f>
```

#### `MODE <mode>`

Accepted values:

- numeric: `0`, `1`, `2`
- aliases:
  - `IDLE`
  - `OPEN`, `OPEN_LOOP`, `OL`
  - `CLOSED`, `CLOSED_LOOP`, `CL`

#### `IMU <0|1|OFF|ON>`

Reads or sets IMU enable state.

#### `STOP`

Sends zero PC command.

#### `DRIVE <v> <w>`

- `v` and `w` are parsed as floats
- clamped to `[-1.0, 1.0]`

`CTRL <v> <w>` is an alias of `DRIVE`.

#### `RATE <hz>`

- valid range: `1..50`

#### `STREAM <ON|OFF> [hz]`

- `STREAM ON`
- `STREAM ON 20`
- `STREAM OFF`

### Streaming Telemetry Line

When ASCII stream is enabled, `PC_Link_StreamOnce()` emits:

```text
TEL,<tick_ms>,<mode>,<src>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<enc_l1_pos>,<enc_l2_pos>,<enc_r1_pos>,<enc_r2_pos>,<vel_l1>,<vel_l2>,<vel_r1>,<vel_r2>,<vbatt>,<pct>
```

Units:

- accel: `g`
- gyro: `dps`
- encoder velocity: `counts/s`
- battery percent: plain percent in range `0..100`

## USART1 Debug And FireWater Telemetry

`USART1` is used for plain text boot/debug output and CSV telemetry.

### Boot / Runtime Text Messages

Examples emitted by firmware:

- reset cause
- MPU6050 init result
- MPU6050 register dump
- IMU calibration start / success / failure
- SSD1306 init result
- PS2 status

### FireWater CSV

When `TELEMETRY_ENABLE = 1`, `main.c` emits one CSV line every `TELEMETRY_INTERVAL_MS`.

Current field order:

1. `tick_ms`
2. `imu_valid`
3. `ax_mg`
4. `ay_mg`
5. `az_mg`
6. `gx_cdps`
7. `gy_cdps`
8. `gz_cdps`
9. `enc0_cps`
10. `enc1_cps`
11. `enc2_cps`
12. `enc3_cps`
13. `v_cmd_x1000`
14. `w_cmd_x1000`
15. `u0_x1000`
16. `u1_x1000`
17. `u2_x1000`
18. `u3_x1000`
19. `roll_x100_deg`
20. `pitch_x100_deg`
21. `yaw_x100_deg`
22. `battery_mv`
23. `battery_pct_x10`
24. `enc_fault_mask`

Notes:

- `mg` means milli-g
- `cdps` means centi-deg/s
- this stream is intended for tools such as VOFA+ FireWater

## ESP-01S Side Expectations

`ESP01S/ESP01S/ESP01S.ino` implements the peer for the same binary protocol.

Observed behavior:

- `SET_DRIVE` and `SET_MODE` may be sent without ACK request
- `GET_STATUS` is sent with ACK request
- COBS framing and constants match firmware:
  - `SOF = 0xA5`
  - `EOF = 0x5A`
  - `VER = 0x01`

## Fault And Diagnostic Semantics

Diagnostic counters are link-local. Fault log is global across links.

Fault severities:

- `0` info
- `1` warn
- `2` error
- `3` fatal

Important implementation detail:

- If the fault ring buffer is full, new records overwrite oldest entries and increment `dropped_fault_count`.

## IMU Data Path

Relevant to serial consumers:

- IMU calibration starts at boot in `main.c`
- calibration now runs for a fixed duration using static detection
- attitude output includes:
  - `roll`, `pitch`, `yaw`
  - quaternion `qw`, `qx`, `qy`, `qz`
  - body-frame linear acceleration
  - world-frame linear acceleration

The current serial protocols do not yet transmit quaternion or world-frame acceleration directly. They are available inside firmware via `Attitude_GetData()`.


