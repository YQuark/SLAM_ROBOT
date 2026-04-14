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
- `IMU_READ_INTERVAL_MS = 20`
- `TELEMETRY_INTERVAL_MS = 200`
- `CMD_TIMEOUT_MS = 300`
- `ACK_TIMEOUT_MS = 80`
- `ACK_RETRY_MAX = 2`
- `FAULT_LOG_CAPACITY = 64`

## Host Odom Contract

The current odometry contract is:

- Translation feedback: firmware-side `v_est` and `w_est`
- Heading feedback: firmware-side fused `yaw_est`
- Gyro auxiliary term: bias-corrected `gz`
- Accelerometer data: debug-only, not for host translation integration

Hard rules for the host:

- The host should use `v_est` and `w_est` as the primary motion feedback terms exposed by the MCU.
- The host should prefer `yaw_est` over integrating raw `gz` by itself.
- The host must check `imu_valid` before treating `yaw_est` / `gz` as high-confidence IMU-assisted heading data.
- The host must check `imu_accel_valid` before trusting any acceleration-derived quantity.
- The host must not integrate `raw_accel_mg[3]` into odometry at this stage.

This contract is meant to make upper-layer mapping and navigation consume the same motion interpretation that the firmware itself uses internally.

## Control Source Correction Policy

Current closed-loop straight-line correction is source-aware:

- PC / host commands may use absolute heading hold.
  - Controlled by `YAW_HOLD_ALLOW_PC = 1`.
  - The controller captures the current fused heading as `yaw_hold` reference when straight driving starts.
  - If IMU attitude is valid, the damping term uses bias-corrected body Z gyro.
  - If IMU is not valid, the damping term falls back to firmware-side `w_est`.
- PS2 and ESP manual commands do not use absolute heading hold by default.
  - Controlled by `YAW_HOLD_ALLOW_PS2 = 0` and `YAW_HOLD_ALLOW_ESP = 0`.
  - They use `straight_balance` for small straight-line assistance.
  - With `STRAIGHT_BALANCE_USE_IMU = 1`, valid IMU data makes this balance loop use bias-corrected body Z gyro first.
  - If IMU is invalid, it falls back to encoder-derived `w_est`.

The default source gates are:

- `YAW_HOLD_ALLOW_PC = 1`
- `YAW_HOLD_ALLOW_PS2 = 0`
- `YAW_HOLD_ALLOW_ESP = 0`
- `STRAIGHT_BALANCE_ALLOW_PC = 0`
- `STRAIGHT_BALANCE_ALLOW_PS2 = 1`
- `STRAIGHT_BALANCE_ALLOW_ESP = 1`

This policy does not change the binary protocol. It only changes how the firmware interprets fresh commands from each source while in `MODE_CLOSED_LOOP`.

## IMU Mount And Static Gravity Gate

The MPU6050 data path maps raw sensor axes into body-frame axes before attitude fusion and calibration checks. Current mount assumptions are configured in `Core/Inc/Drivers/robot_config.h`:

- body X comes from sensor Y
- body Y comes from sensor Z
- body Z comes from sensor X

During static calibration, firmware now checks the mapped body-frame average acceleration:

- body Z must be at least `IMU_CALIB_BODY_Z_MIN_G`
- body X/Y magnitude must stay below `IMU_CALIB_BODY_XY_MAX_G`

If the mapped static gravity direction is not plausible, accelerometer calibration is rejected and `imu_accel_valid` remains `0`. This protects host consumers from treating a wrong IMU mounting direction as a trusted acceleration source.

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
- `0x13` `SET_RAW`
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
  18. `cmd_semantics` `u8`
  19. `raw_left_q15` `i16`
  20. `raw_right_q15` `i16`

Notes:

- `v_cmd_q15` and `w_cmd_q15` are closed-loop velocity commands scaled by `32767`.
- `v_est_q15` and `w_est_q15` are firmware-side estimated chassis linear/angular velocity, also scaled by `32767`.
- `gz_dps_x10` is bias-corrected when IMU attitude is valid; otherwise it falls back to raw converted gyro rate.
- `yaw_est_x100_deg` is the fused heading used by firmware and is the preferred host heading input.
- `enc_fault_mask` exposes encoder fault state bits for host-side diagnostics.
- `enc_vel_cps` order follows `ENC_L1`, `ENC_L2`, `ENC_R1`, `ENC_R2`.
- `raw_accel_mg[3]` is debug-only and must not be used for translational odometry yet.
- `imu_valid` means yaw / gyro assistance is available.
- `imu_accel_valid` means accelerometer data passed static calibration and current runtime gating.
- `cmd_semantics`: `0=none`, `1=velocity`, `2=raw_left_right`
- `raw_left_q15` / `raw_right_q15` are meaningful only when `cmd_semantics=2`

Host bridge interpretation:

- `v_est_q15` and `w_est_q15` should be decoded into physical units using the host-side `max_linear` and `max_angular` parameters.
- `yaw_est_x100_deg` should be treated as the MCU's preferred heading output.
- `gz_dps_x10` is useful for IMU messages, damping, and diagnostics, but is not a replacement for `yaw_est_x100_deg`.
- `raw_accel_mg[3]` is included so the host can inspect IMU axis orientation and health, not to estimate planar displacement.

### `SET_DRIVE` `0x10`

- Request payload:
  1. `v_q15` `i16`
  2. `w_q15` `i16`

Behavior:

- Semantic: closed-loop normalized velocity command
- `LINK_ID_ESP` updates `RobotControl_SetCmd_ESP`
- `LINK_ID_PC` updates `RobotControl_SetCmd_PC`
- If current chassis mode is not `MODE_CLOSED_LOOP`, firmware returns `NACK` with `LINK_ERR_CTRL_MODE_REJECT`

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

### `SET_RAW` `0x13`

- Request payload:
  1. `left_q15` `i16`
  2. `right_q15` `i16`

Behavior:

- Semantic: open-loop raw left/right normalized motor output
- `LINK_ID_ESP` and `LINK_ID_PC` both route to `RobotControl_SetOpenLoopCmd(left, right, src, now_ms)`
- If current chassis mode is not `MODE_OPEN_LOOP`, firmware returns `NACK` with `LINK_ERR_CTRL_MODE_REJECT`

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
OK CMDS: PING, HELP, STATUS, DEBUG, MODE <0|1|2>, DRIVE <v> <w>, CTRL <v> <w>, RAW <l> <r>, MOTOR <l> <r>, STOP, IMU <0|1>, STREAM <ON|OFF> [hz], RATE <hz>
```

#### `STATUS`

Response format:

```text
OK STATUS mode=<u> src=<u> cmd_sem=<u> v_cmd_x1000=<i> w_cmd_x1000=<i> raw_l_x1000=<i> raw_r_x1000=<i> v_est_x1000=<i> w_est_x1000=<i> yaw_x100=<i> vbat_mv=<i> imu_en=<u> imu_ok=<u> imu_acc_ok=<u> gz_cdps=<i> enc_fault=0xNN motor_ovr=<u>
```

Host usage notes:

- `v_est_x1000` / `w_est_x1000` are the odometry-ready motion estimates.
- `yaw_x100` is the fused heading that should be preferred over integrating raw gyro on the host.
- `gz_cdps` is the bias-corrected Z gyro when IMU attitude is valid.
- `cmd_sem=0/1/2` means `none/velocity/raw`
- `v_cmd_x1000` / `w_cmd_x1000` are only meaningful when `cmd_sem=1`
- `raw_l_x1000` / `raw_r_x1000` are only meaningful when `cmd_sem=2`
- `motor_ovr=1` now means chassis is currently in `MODE_OPEN_LOOP`; it no longer means legacy PC-side direct motor bypass.
- `pc_ctrl` / `esp_ctrl` now mean the corresponding velocity command is still fresh within `CMD_TIMEOUT_MS`, not merely that the link sent control traffic recently.

#### `DRIVE <v> <w>` / `CTRL <v> <w>`

- Semantic: closed-loop normalized chassis velocity command
- Valid only in `MODE_CLOSED_LOOP`
- If current mode is not closed loop, firmware responds:

```text
ERR DRIVE requires MODE CLOSED
```

#### `RAW <left> <right>` / `MOTOR <left> <right>`

- Semantic: open-loop raw left/right normalized motor output
- Valid only in `MODE_OPEN_LOOP`
- If current mode is not open loop, firmware responds:

```text
ERR RAW requires MODE OPEN
```

- Success response:

```text
OK RAW l_x1000=<i> r_x1000=<i>
```

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

## ROS2 Bridge Alignment

The current upper-machine bridge implementation is `Ros2_Slam/src/stm32_robot_bridge/stm32_robot_bridge/bridge_node.py`.

Its current actual behavior is:

- `/cmd_vel` is encoded into `SET_DRIVE`
- startup and probing use `GET_STATUS`
- `GET_STATUS` is the primary status source
- `/odom.twist` uses `v_est_q15` and `w_est_q15`
- `/odom.pose` integrates the bridge's own `(x, y, yaw)` state
- when `use_status_yaw=true`, the bridge snaps its internal `yaw` to `yaw_est_x100_deg` on each fresh status frame
- when `use_status_yaw=false` (current default), the bridge still uses `v_est/w_est`, but `yaw` continues by integrating `w_est`
- `/imu/data.orientation` uses `yaw_est_x100_deg` only when `feedback_imu_valid` is true
- `/imu/data.angular_velocity.z` uses bias-corrected `gz`
- `/imu/data.linear_acceleration` uses `raw_accel_mg[3]` converted to SI units

Implication:

- If the host wants the MCU heading to directly shape odometry heading, it must explicitly set `use_status_yaw=true`.
- If `use_status_yaw=false`, the bridge is still using MCU `v_est/w_est`, but not fully trusting MCU `yaw_est` for `/odom.pose`.

## ESP01S HTTP Bridge Contract

`ESP01S/ESP01S/ESP01S.ino` exposes a small HTTP control surface that now follows the same control semantics as the STM32 firmware.

### Endpoints

- `GET /mode?m=<0|1|2>`
  - `0` idle
  - `1` open-loop raw output
  - `2` closed-loop velocity
  - bridge waits for STM32 ACK before reporting success
- `GET /cmd?v=<float>&w=<float>`
  - Semantic: closed-loop velocity command
  - Valid only when current mode is `2`
  - Returns HTTP `409` if chassis is not in closed-loop mode
- `GET /raw?l=<float>&r=<float>`
  - Semantic: open-loop left/right raw output
  - Valid only when current mode is `1`
  - Returns HTTP `409` if chassis is not in open-loop mode
- `GET /status`
  - Returns cached JSON status assembled from the most recent binary `GET_STATUS` response
- `GET /health`
  - Returns bridge diagnostics

### `/status` JSON Notes

Important fields:

- `mode`
- `mode_name`
- `control_api`
- `cmd_space`
- `src`
- `v`
- `w`
- `raw_left`
- `raw_right`
- `imu`
- `enc`

`control_api` is the bridge-side hint for which HTTP control endpoint should currently be used:

- `none` for idle
- `raw` for open-loop
- `drive` for closed-loop

`cmd_space` reports the current command interpretation in the most recent STM32 status frame:

- `none`
- `velocity`
- `raw`

### HTTP Hold And Serial Keep-Alive Timing

STM32 command freshness is bounded by `CMD_TIMEOUT_MS = 300ms`. The ESP bridge keeps active HTTP control below that age limit:

- serial control frame minimum period: `DRIVE_SEND_PERIOD_MS = 40ms`
- unchanged command refresh period: `CONTROL_REFRESH_MS = 100ms`
- browser hold resend period: `HOLD_KEEP_MS = 120ms`
- HTTP idle stop guard: `CMD_IDLE_STOP_MS = 700ms`
- HTTP duplicate filter window: `CMD_DUP_FILTER_MS = 60ms`

Important behavior:

- `/cmd` refreshes closed-loop velocity commands.
- `/raw` refreshes open-loop left/right raw commands.
- Repeated identical HTTP commands still update the HTTP active timestamp before returning `OK`.
- If HTTP commands stop for longer than `CMD_IDLE_STOP_MS`, the bridge marks both pending velocity and raw commands as zero.
- Button release, page blur, or page hide sends an immediate zero command; the 700ms idle guard is only a fallback for lost release events.

When the web UI switches between closed-loop and open-loop views, the long-press handler stores both drive-space and raw-space values and sends the pair appropriate for the latest `status.mode`.

## Current Mode Policy

Current firmware startup / local-control policy is:

- boot default mode is `MODE_CLOSED_LOOP`
- PS2 local movement input from `MODE_IDLE` auto-enters `MODE_CLOSED_LOOP`
- open-loop must be selected explicitly through `MODE 1`, binary `SET_MODE(1)`, or `GET /mode?m=1`

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
- static accelerometer validity also requires mapped body-frame gravity to match the configured mount direction
- manual-source straight balance uses body Z gyro first when IMU is valid, but still falls back to encoder `w_est`

Quaternion and world-frame linear acceleration remain available inside firmware via `Attitude_GetData()`, but they are not part of the current host odometry contract.


