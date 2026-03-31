# 2026-03-31 PS2 Host Stability Requirements

## Background

The user reports that when the board is connected to the host computer for mapping/navigation, the PS2 controller often appears to disconnect and the system may appear to restart. Hardware has been rechecked by the user. The MPU6050 module has also changed from a soldered module to a pin-header module, and the user expects many IMU readings to now be normal.

The current firmware has three relevant runtime properties:

1. `PS2` communication is implemented as GPIO bit-bang timing in `Core/Src/Drivers/ps2.c`.
2. `USART3` host communication remains active during host-side mapping/navigation and can still generate frequent receive interrupts and blocking transmit activity.
3. `MPU6050` and `SSD1306` share `I2C1`, and their HAL operations use long blocking timeouts in the control loop.

## Problem Statement

During host-connected mapping/navigation, PS2 control becomes unstable and may show invalid-mode / disconnect behavior. In the same operating window, the board can appear to restart. The firmware must be hardened against timing interference and long blocking stalls that plausibly cause these symptoms.

## Functional Requirements

1. PS2 timing integrity
   `PS2` transfer timing must be protected from unrelated UART interrupt preemption during a single frame exchange so that host serial traffic is less likely to corrupt bit-bang sampling.

2. I2C stall containment
   Runtime `MPU6050` and `SSD1306` I2C transactions must use tighter timeout bounds so that transient bus issues do not monopolize the main loop for large fractions of the watchdog window.

3. Watchdog-risk reduction
   Normal runtime paths that involve `USART3`, `MPU6050`, `SSD1306`, and PS2 must be less likely to create long blocking chains that lead to apparent resets.

4. Behavior preservation
   Existing `USART3` host command behavior and existing PS2 control semantics must remain compatible.

## Non-Goals

1. Rewriting the whole host serial stack to DMA or RTOS tasks.
2. Implementing a full generic I2C bus-recovery state machine.
3. Using the MPU6050 connector change alone as justification to retune all IMU algorithms.

## Acceptance Criteria

1. `PS2` frame exchange code protects its bit-level critical timing against asynchronous interrupt preemption.
2. Runtime I2C access paths for `MPU6050` and `SSD1306` no longer wait up to 100 ms per low-level transaction.
3. The code builds logically cleanly and the modified paths are traceable to the reported failure mode.
4. Verification notes explicitly state what was checked locally and what remains a field validation item.
