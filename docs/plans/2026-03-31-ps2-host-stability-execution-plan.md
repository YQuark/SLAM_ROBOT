# 2026-03-31 PS2 Host Stability Execution Plan

## Internal Grade Decision

`L`

This is a bounded firmware-hardening task with a single coherent write scope and no need for parallel implementation waves.

## Execution Units

1. Root-cause confirmation
   Inspect `main.c`, `ps2.c`, `pc_link.c`, `mpu6050.c`, `ssd1306.c`, and watchdog settings to validate the timing/stall hypothesis.

2. PS2 timing hardening
   Add a minimal critical section around PS2 frame transfer so `USART3` receive interrupts do not split the bit-bang waveform mid-frame.

3. I2C stall reduction
   Reduce low-level I2C transaction timeouts for runtime MPU6050/OLED paths to tighten worst-case blocking windows.

4. Verification
   Re-read the modified paths and run the project build if the toolchain is available.

## Ownership Boundaries

- `Core/Src/Drivers/ps2.c`: PS2 timing protection.
- `Core/Src/Drivers/mpu6050.c`: IMU I2C timeout tightening.
- `Core/Src/Drivers/ssd1306.c`: OLED I2C timeout tightening.

## Verification Commands

- `rg -n "disable_irq|enable_irq|HAL_I2C_Mem_Read|HAL_I2C_Mem_Write|HAL_I2C_Master_Transmit" Core/Src/Drivers/ps2.c Core/Src/Drivers/mpu6050.c Core/Src/Drivers/ssd1306.c`
- `cmake --build build`

## Delivery Acceptance Plan

Accept completion wording only if:

1. The intended code paths were changed as planned.
2. The modified code is internally consistent.
3. Build verification either passes or any build blocker is explicitly reported.

## Rollback Rules

If the PS2 critical section causes unacceptable latency side effects, revert only that change and preserve the I2C timeout hardening.

## Phase Cleanup Expectations

Emit:

- `outputs/runtime/vibe-sessions/20260331-ps2-host-stability/phase-plan_execute.json`
- `outputs/runtime/vibe-sessions/20260331-ps2-host-stability/cleanup-receipt.json`
