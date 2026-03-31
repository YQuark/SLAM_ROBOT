# 2026-03-31 IMU Motion Stall Execution Plan

## Internal Grade Decision

`L`

The work is localized to the shared `I2C1` path and its two main consumers.

## Execution Units

1. Add `I2C1` bus recovery helper.
2. Wire recovery-assisted retry into MPU6050 runtime read/write paths.
3. Wire recovery-assisted retry into SSD1306 runtime write paths.
4. Re-read changed paths and record verification limits.

## Ownership Boundaries

- `Core/Inc/i2c.h`
- `Core/Src/i2c.c`
- `Core/Src/Drivers/mpu6050.c`
- `Core/Src/Drivers/ssd1306.c`

## Verification Commands

- `rg -n "I2C_BusRecover|HAL_I2C_Mem_Read|HAL_I2C_Mem_Write|HAL_I2C_Master_Transmit" Core/Src Core/Inc`
- `cmake --build build`

## Delivery Acceptance Plan

Completion wording remains limited unless build verification and hardware shake testing both happen.

## Rollback Rules

If the recovery routine proves unsafe for unrelated I2C traffic, revert the retry wiring first and preserve any harmless timeout reductions.

## Phase Cleanup Expectations

Emit:

- `outputs/runtime/vibe-sessions/20260331-imu-motion-stall/phase-plan_execute.json`
- `outputs/runtime/vibe-sessions/20260331-imu-motion-stall/cleanup-receipt.json`
