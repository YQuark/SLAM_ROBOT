# 2026-03-31 IMU Motion Stall Requirements

## Background

The user reports that when the robot is shaken strongly, the system can still freeze. The symptom now appears more correlated with the MPU6050 path than with host serial traffic alone.

Code inspection shows:

1. `Attitude_Update()` does not contain an obvious motion-triggered infinite loop. Under bad acceleration it mainly drops accelerometer trust and keeps running.
2. `MPU6050` and `SSD1306` both use `I2C1`.
3. A motion-induced connector/contact glitch on the plug-in MPU6050 can plausibly create I2C read/write errors or a stuck bus that affects the main loop.

## Problem Statement

When motion perturbs the IMU path, the firmware must avoid hanging indefinitely on shared `I2C1` transactions. A bounded recovery path is required so the robot degrades and recovers instead of appearing dead.

## Functional Requirements

1. Runtime I2C1 recovery
   The firmware shall provide a bounded `I2C1` bus recovery routine that can be called after runtime MPU6050 or SSD1306 I2C failures.

2. MPU6050 bounded retry
   Runtime MPU6050 register and bulk reads shall attempt one recovery-assisted retry after an I2C failure.

3. SSD1306 bounded retry
   Runtime SSD1306 command/data writes shall attempt one recovery-assisted retry after an I2C failure.

4. Shared-bus containment
   A transient MPU6050-side bus fault shall be less likely to leave the whole `I2C1` bus unusable until power cycle.

## Non-Goals

1. Guaranteeing recovery from all physical disconnects or power drops.
2. Replacing the MPU6050 hardware module.
3. Changing IMU fusion thresholds as the primary mitigation.

## Acceptance Criteria

1. `I2C1` exposes a callable recovery routine.
2. MPU6050 runtime accessors invoke bounded recovery and retry on failure.
3. SSD1306 runtime accessors invoke bounded recovery and retry on failure.
4. Verification notes explicitly state that hardware validation is still required.
