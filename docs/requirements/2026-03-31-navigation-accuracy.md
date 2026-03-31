# 2026-03-31 Navigation Accuracy Requirements

## Background

The current firmware no longer exhibits the previously reported PS2 motion stutter after recent rollback work. The next reported quality issues are:

1. Heading / yaw behavior still feels imprecise.
2. Host-side odometry and resulting maps are inaccurate, especially around right-angle turns.
3. The chassis does not track straight enough under nominal straight-line commands.
4. Host communication must be robust enough to keep transmitting data continuously while remaining operational for control and status exchange.

## Problem Statement

The firmware currently exposes motion and communication behavior that is functionally usable but not accurate enough for reliable mapping/navigation. The control and telemetry paths must be refined so heading, straight-line tracking, and host-facing motion data are more coherent.

## Functional Requirements

1. Heading path coherence
   Runtime heading estimation and heading-hold behavior must use a deliberate configuration rather than relying on a disabled IMU path by default.

2. Straight-line tracking refinement
   The control layer must better suppress left/right drift during nominal straight motion without destabilizing turn response.

3. Host odometry improvement
   The telemetry/protocol path sent to the host must expose motion quantities that are more appropriate for odometry and navigation, with particular attention to turn geometry.

4. Continuous host communication
   The host link must continue streaming telemetry while preserving command/status protocol behavior and without reintroducing blocking-loop regressions.

## Non-Goals

1. Rewriting the host SLAM algorithm.
2. Replacing the full wheel odometry model with a complete kinematic calibration pipeline.
3. Claiming final navigation accuracy without board-side validation.

## Acceptance Criteria

1. The runtime configuration and control logic clearly enable and use the intended heading signal path.
2. Straight-line control receives at least one firmware-side refinement targeted at left/right drift reduction.
3. The host-facing telemetry/protocol surface is made more explicit and more suitable for odometry use.
4. Continuous host telemetry remains non-blocking in the firmware path.
