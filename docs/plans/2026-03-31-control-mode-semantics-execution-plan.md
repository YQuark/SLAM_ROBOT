# 2026-03-31 Control Mode Semantics Execution Plan

## Internal Grade Decision

`L`

This is a bounded implementation task with one coherent write scope and no need for parallel execution.

## Execution Units

1. Commit the current workspace baseline.
2. Refactor the control core so open-loop mode consumes only explicit raw-duty commands.
3. Update PC ASCII commands and binary link protocol to expose raw-duty commands distinctly from velocity commands.
4. Re-read the modified paths and attempt a build if the toolchain exists.

## Ownership Boundaries

- `Core/Inc/Drivers/robot_control.h`
- `Core/Src/Drivers/robot_control.c`
- `Core/Inc/Drivers/link_proto.h`
- `Core/Src/Drivers/link_proto.c`
- `Core/Src/Drivers/pc_link.c`

## Verification Commands

- `rg -n "SetOpenLoopCmd|MODE_OPEN_LOOP|LINK_MSG_CMD_SET_RAW|RAW|MOTOR" Core/Src Core/Inc`
- `cmake --build build`

## Delivery Acceptance Plan

Accept completion wording only if:

1. The semantic split is visible in code.
2. The host commands clearly reflect the split.
3. Build verification either passes or the blocker is explicitly reported.

## Rollback Rules

If compatibility fallout is too high, preserve the core open-loop isolation and restore only command aliases as needed.

## Phase Cleanup Expectations

Emit:

- `outputs/runtime/vibe-sessions/20260331-control-mode-semantics/phase-plan_execute.json`
- `outputs/runtime/vibe-sessions/20260331-control-mode-semantics/cleanup-receipt.json`
