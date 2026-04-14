# IMU Axis Remap And Heading Control Execution Plan

日期：2026-04-02

## Internal Grade

`L`

原因：

- 改动集中在 STM32 下位机单工程
- 需要串行完成：安装映射、控制切换、验证
- 这是控制基础语义修正，不适合并行分拆

## Steps

1. 冻结 IMU 安装轴修正需求
2. 在 `mpu6050` 中加入固定车体轴映射
3. 将 PC 控制源直行辅助切到航向保持
4. 将 PS2/ESP 控制源直行辅助切到轻量直行平衡，IMU 有效时使用车体 Z 轴角速度
5. 加入映射后静态重力方向校验
6. 编译验证并记录剩余风险

## Ownership Boundaries

- `Core/Inc/Drivers/robot_config.h`
- `Core/Src/Drivers/mpu6050.c`
- `Core/Src/Drivers/robot_control.c`

## Verification Commands

- `cmake --build /mnt/d/Document/Work/projects/Clion/build/Debug`

## Delivery Acceptance Plan

- IMU 映射逻辑清晰且可解释
- 控制源分层纠偏逻辑清晰且可解释
- 静态重力方向校验能阻止错误安装方向被标记为加速度可信
- 构建通过

## Completion Language Rules

- 未经编译验证，不允许说“完成”
- 必须保留“需要实车复测”的风险说明

## Rollback Rules

- 若映射方向明显错误，优先调整映射符号，不回退整个控制结构

## Phase Cleanup

- 留下 requirement / plan / runtime receipt
- 不新增临时测试文件
