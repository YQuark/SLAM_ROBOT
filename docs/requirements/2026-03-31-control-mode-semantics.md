# 2026-03-31 Control Mode Semantics Requirements

## Background

The user requested that the current code be committed first, then asked for the control framework changes previously recommended: make the meaning of open-loop and closed-loop control explicit and stop mixing velocity commands with raw motor output semantics.

## Problem Statement

The current firmware has a real open-loop branch and a real closed-loop branch, but host commands still blur their meaning. In particular, closed-loop velocity commands and raw motor commands are not cleanly separated at the system interface level.

## Functional Requirements

1. Open-loop isolation
   `MODE_OPEN_LOOP` must consume only explicit raw left/right duty commands.

2. Closed-loop isolation
   Velocity `DRIVE` commands must remain a closed-loop path and must not silently act as open-loop motor output.

3. Host path clarity
   Both ASCII host control and binary link protocol must expose a distinct raw-duty command path.

4. Compatibility discipline
   Existing host users should retain a practical migration path, for example by keeping `MOTOR` as an alias of the new raw-duty concept.

## Non-Goals

1. Full protocol redesign.
2. Control retuning.
3. Major source-arbitration redesign.

## Acceptance Criteria

1. The code path for `MODE_OPEN_LOOP` no longer depends on generic `SetCmd_PC/ESP`.
2. PC ASCII commands distinguish closed-loop velocity from raw duty.
3. Binary host control adds an explicit raw-duty command path.
