# 2026-03-31 Chassis Control Architecture Requirements

## Background

The user asks for an opinion on the current chassis control framework, especially whether the open-loop and closed-loop distinction is meaningful and how the whole control stack should be understood.

## Problem Statement

The current firmware contains a real open-loop branch, a real closed-loop branch, and separate command sources from PS2, ESP, and PC. However, the semantics between control mode, command source, and host protocol are not cleanly separated, which makes the system harder to reason about and may make mode behavior feel inconsistent.

## Functional Requirements

1. Explain the current framework using the actual code structure.
2. Identify the architectural ambiguities or weak points.
3. Recommend a cleaner target structure for future work.

## Non-Goals

1. Implementing the redesign in this turn.
2. Claiming quantitative control improvement without testing.

## Acceptance Criteria

1. The assessment distinguishes mode semantics from command transport semantics.
2. The assessment is grounded in current source files.
3. The recommendation is actionable for a later implementation turn.
