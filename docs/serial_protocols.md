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
- Typical traffic:
  - boot log
  - MPU6050 status
  - IMU calibration messages
  - optional FireWater CSV telemetry

### USART2

- Purpose: ESP-01S bridge link
- Firmware module: `ESP_Link_*`
- Protocol: binary framed protocol only

### USART3

- Purpose: PC host link
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

## Host Odom Contract

The current odometry contract is encoder-dominated.

- Primary odometry source: encoder-derived `v_est` and `w_est`
- Heading assist: fused `yaw_est` and bias-corrected `gz`
- IMU heading gating: host must check `imu_valid` before trusting IMU-assisted heading data
- IMU acceleration gating: host must check `imu_accel_valid` before using any acceleration-derived quantity
- Accelerometer policy: raw acceleration is debug-only and must not be integrated into host odometry at this stage

This means the host should integrate wheel motion as the primary source and use IMU only to stabilize heading, not to estimate translation.

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
  11. `v_est_q15` `i16`
  12. `w_est_q15` `i16`
  13. `enc_fault_mask` `u8`
  14. `enc_vel_cps[4]` `i16 x 4`
  15. `yaw_est_x100_deg` `i16`
  16. `raw_accel_mg[3]` `i16 x 3`
  17. `imu_accel_valid` `u8`

Notes:

- `v_cmd_q15` and `w_cmd_q15` are normalized command values scaled by `32767`.
- `v_est_q15` and `w_est_q15` are firmware-side estimated chassis linear/angular velocity, also scaled by `32767`.
- `gz_dps_x10` is bias-corrected when IMU attitude is valid; otherwise it falls back to raw converted gyro rate.
- `yaw_est_x100_deg` is the fused heading used by firmware and is the preferred host heading input.
- `enc_fault_mask` exposes encoder fault state bits for host-side diagnostics.
- `enc_vel_cps` order follows `ENC_L1`, `ENC_L2`, `ENC_R1`, `ENC_R2`.
- `raw_accel_mg[3]` is debug-only and must not be used for translational odometry yet.
- `imu_valid` means yaw / gyro assistance is available.
- `imu_accel_valid` means accelerometer data passed static calibration and current runtime gating.

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

### `GET_FAULT_LOG` `0x32`

- Request payload:
  - byte 0 optional: `cursor`
  - byte 1 optional: `max_records`

### `CLEAR_DIAG` `0x31`

- Request payload: empty
- Effect: clears snapshot counters for the current link only

### `CLEAR_FAULT_LOG` `0x33`

- Request payload: empty
- Effect: clears global fault ring buffer

## USART3 ASCII Console

ASCII parsing is implemented in `Core/Src/Drivers/pc_link.c`.

### Commands

#### `PING`

Response:

```text
OK PONG
```

#### `HELP`

Response:

```text
OK CMDS: PING, HELP, STATUS, DEBUG, MODE <0|1|2>, DRIVE <v> <w>, CTRL <v> <w>, MOTOR <l> <r>, STOP, IMU <0|1>, STREAM <ON|OFF> [hz], RATE <hz>
```

#### `STATUS`

Response format:

```text
OK STATUS mode=<u> src=<u> v_cmd_x1000=<i> w_cmd_x1000=<i> v_est_x1000=<i> w_est_x1000=<i> yaw_x100=<i> vbat_mv=<i> imu_en=<u> imu_ok=<u> imu_acc_ok=<u> gz_cdps=<i> enc_fault=0xNN motor_ovr=<u>
```

Host usage notes:

- `v_est_x1000` / `w_est_x1000` are the odometry-ready motion estimates.
- `yaw_x100` is the fused heading that should be preferred over integrating raw gyro on the host.
- `gz_cdps` is the bias-corrected Z gyro when IMU attitude is valid.

#### `DEBUG`

Response format:

```text
OK DEBUG mode=<u> src=<u> ref=<i>,<i>,<i>,<i> meas=<i>,<i>,<i>,<i> u_x1000=<i>,<i>,<i>,<i> ccr=<u>,<u>,<u>,<u> dir=<u>,<u>,<u>,<u> fault=0xNN
```

#### `STREAM <ON|OFF> [hz]`

When ASCII stream is enabled, `PC_Link_StreamOnce()` emits:

```text
TEL,<tick_ms>,<mode>,<src>,<imu_valid>,<imu_accel_valid>,<v_est_x1000>,<w_est_x1000>,<yaw_est_x100_deg>,<gz_bias_cdps>,<vel_l1_cps>,<vel_l2_cps>,<vel_r1_cps>,<vel_r2_cps>,<enc_fault_mask>,<vbatt_mv>,<pct_x10>,<raw_ax_mg>,<raw_ay_mg>,<raw_az_mg>
```

Units:

- `v_est_x1000`: normalized linear velocity estimate scaled by `1000`
- `w_est_x1000`: normalized angular velocity estimate scaled by `1000`
- `yaw_est_x100_deg`: fused heading in degrees scaled by `100`
- `gz_bias_cdps`: bias-corrected Z gyro in centi-deg/s
- `imu_valid`: yaw / gyro assistance availability flag
- `imu_accel_valid`: accelerometer availability flag
- encoder velocity: `counts/s`
- battery voltage: `mV`
- battery percent: `x10`
- raw accel: `mg`, debug-only

ASCII odom consumer notes:

- Odom-ready fields are `imu_valid`, `v_est_x1000`, `w_est_x1000`, `yaw_est_x100_deg`, `gz_bias_cdps`, wheel velocities, and `enc_fault_mask`.
- `raw_ax_mg`, `raw_ay_mg`, and `raw_az_mg` are transmitted only for diagnosis.
- If IMU calibration is incomplete or invalid, `imu_valid=0` and `gz_bias_cdps=0`.
- If the accelerometer fails static calibration or later runtime gating, `imu_accel_valid=0` even when `imu_valid=1`.

## USART1 Debug And FireWater Telemetry

`USART1` is used for plain text boot/debug output and CSV telemetry.

FireWater CSV remains a debug stream. It is not the host odometry contract.

## IMU Data Path

Relevant to serial consumers:

- IMU calibration starts at boot in `main.c`
- calibration runs for a fixed duration using static detection
- attitude output includes `roll`, `pitch`, `yaw`, quaternion, body-frame linear acceleration, and world-frame linear acceleration

Current policy:

- encoder odometry is primary
- IMU is used for heading assistance only
- bias-corrected gyro is exported for host use
- accelerometer values are debug-only until the abnormal acceleration issue is resolved
- firmware now distinguishes `imu_valid` (heading usable) from `imu_accel_valid` (acceleration usable)

Quaternion and world-frame linear acceleration remain available inside firmware via `Attitude_GetData()`, but they are not part of the current host odometry contract.


