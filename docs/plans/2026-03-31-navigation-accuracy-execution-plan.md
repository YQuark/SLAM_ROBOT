# 2026-03-31 Navigation Accuracy Execution Plan

## Internal Grade Decision

`L`

This is a bounded firmware refinement task with one write scope and sequential dependency across diagnosis, control changes, telemetry changes, and verification.

## Execution Units

1. Diagnose the current heading, odometry, drift, and protocol paths in `robot_control.c`, `main.c`, `pc_link.c`, `link_proto.c`, and `robot_config.h`.
2. Refine runtime control configuration for heading participation and straight-line behavior.
3. Improve host-facing telemetry / protocol fields so the motion contract is clearer for mapping/navigation.
4. Re-read the modified paths and run the build if the toolchain exists.

## Ownership Boundaries

- `Core/Inc/Drivers/robot_config.h`: runtime tuning and communication constants.
- `Core/Src/Drivers/robot_control.c`: heading hold and straight-line behavior.
- `Core/Src/Drivers/pc_link.c`: streaming telemetry payload.
- `Core/Src/Drivers/link_proto.c`: binary protocol payload coherence.

## Verification Commands

- `rg -n "USE_IMU_DEFAULT|YAW_HOLD|TEL,|append_status_payload|LINK_UART_TX_TIMEOUT_MS" Core/Inc/Drivers/robot_config.h Core/Src/Drivers/robot_control.c Core/Src/Drivers/pc_link.c Core/Src/Drivers/link_proto.c`
- `cmake --build build`

## Delivery Acceptance Plan

Accept completion wording only if:

1. The intended control and telemetry paths were changed.
2. The resulting runtime behavior is internally coherent on static review.
3. Build verification either passes or the blocker is explicitly reported.

## Rollback Rules

If heading-related changes worsen drivability, revert the tuning/configuration layer first and preserve non-blocking communication fixes.

## Phase Cleanup Expectations

Emit:

- `outputs/runtime/vibe-sessions/20260331-navigation-accuracy/phase-plan_execute.json`
- `outputs/runtime/vibe-sessions/20260331-navigation-accuracy/cleanup-receipt.json`
