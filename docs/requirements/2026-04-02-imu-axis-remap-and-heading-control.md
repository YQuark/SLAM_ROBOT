# IMU Axis Remap And Heading Control Requirements

日期：2026-04-02

## Goal

修复 STM32 下位机对垂直安装 MPU6050 的错误轴向解释，并按控制源区分直行纠偏策略：上位机导航命令使用航向保持，人工遥控命令使用轻量直行平衡。

## Deliverable

- 在 IMU 数据链路中增加“传感器轴到车体轴”的安装映射
- 让姿态融合与下游控制都使用映射后的车体坐标
- 静态校准时检查映射后的车体重力方向，防止错误安装方向被标记为加速度可信
- 让 `PC` 控制源使用 `yaw_hold`
- 让 `PS2/ESP` 控制源默认不使用绝对航向锁定，改用直行平衡
- 直行平衡在 IMU 有效时优先使用车体 Z 轴角速度，IMU 无效时回退编码器 `w_est`
- 编译级验证结果

## Constraints

- 不修改上位机协议格式
- 不删除轮速 PI 闭环
- 不引入新的外部依赖
- 仅做当前已知安装姿态下的固定映射，不做自动安装检测

## Acceptance Criteria

- `MPU6050_Convert()` 产出的姿态输入已是车体坐标，而不是传感器原始坐标
- 静态加速度校准会校验车体 `+Z` 重力方向，异常时 `imu_accel_valid=0`
- `YAW_HOLD_ALLOW_PC=1`，`YAW_HOLD_ALLOW_PS2=0`，`YAW_HOLD_ALLOW_ESP=0`
- `STRAIGHT_BALANCE_ALLOW_PS2=1`，`STRAIGHT_BALANCE_ALLOW_ESP=1`
- 直行平衡在 IMU 有效时不再以编码器 `w_est` 为主纠偏量
- 工程可编译通过

## Product Acceptance Criteria

- 垂直安装的 IMU 能输出物理语义正确的航向相关量
- 上位机直行辅助可使用航向保持，人工遥控不会被强制拉回旧绝对航向
- 滑移场景下，人工直行扶正优先参考 IMU 角速度，而不是被轮速打滑误导
- 后续只需在正确轴向基础上继续小范围调参

## Manual Spot Checks

- 静止时车体 `body_z` 对应加速度接近 `+1g`
- 手动转动车头时 `gz_dps` 应反映车体偏航角速度
- PC 控制源直行时进入 `yaw_hold`
- PS2/ESP 控制源直行时进入 `straight_balance`
- IMU 有效时 `straight_balance` 使用 bias-corrected `gz`

## Completion Language Policy

- 只有代码修改完成并且至少一轮编译通过后，才允许使用“已完成”
- 未做实车复测，不允许声称“航向问题已完全解决”

## Delivery Truth Contract

- 本次交付只声称“修复了 IMU 安装轴解释，并按控制源分层实现航向保持/直行平衡”
- 不声称“最终标定已完成”

## Non-Goals

- 不改上位机导航算法
- 不做 IMU 自动安装识别
- 不处理其他非 IMU 相关安全问题

## Inferred Assumptions

- 当前安装状态下，车体竖直轴对应传感器 `+X`
- 剩余两个车体轴采用与传感器右手系一致的循环映射
- PC 命令主要服务导航/上位机闭环，PS2/ESP 命令主要服务人工遥控体验
