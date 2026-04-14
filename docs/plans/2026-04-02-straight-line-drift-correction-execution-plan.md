# Straight-Line Drift Correction Execution Plan

日期：2026-04-02

## Internal Grade

`L`

原因：

- 问题集中在单个 STM32 控制器工程
- 需要基于现有控制结构做有限修正，而不是并行重构
- 最重要的是快速形成一轮可验证、可回退的实车修正版本

## Steps

1. 冻结左偏问题意图与边界
2. 对 PS2/ESP 控制源启用并收紧直线平衡回路
3. 保持 PC 控制源由航向保持负责直行辅助
4. 让直线平衡在 IMU 有效时优先使用车体 Z 轴角速度，IMU 无效时回退编码器估计
5. 对静态 trim 做保守调整
6. 编译验证
7. 输出剩余实车调参建议

## Ownership Boundaries

- `Core/Inc/Drivers/robot_config.h`
- `Core/Src/Drivers/robot_control.c`（仅当需要）

## Verification Commands

- `cmake --build /mnt/d/Document/Work/projects/Clion/build/Debug`

## Delivery Acceptance Plan

- 直线纠偏逻辑已启用
- 控制源分层清楚：PC 航向保持，PS2/ESP 直线平衡
- 参数改动可解释
- 构建通过

## Completion Language Rules

- 仅在完成编译验证后允许说“已完成”
- 必须明确说明仍需要实车复测

## Rollback Rules

- 若调整导致明显控制不稳，优先回退参数，不回退运动学结构

## Phase Cleanup

- 留下 requirement / plan / runtime receipt
- 不保留临时测试文件
