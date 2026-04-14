# Straight-Line Drift Correction Requirements

日期：2026-04-02

## Goal

针对 STM32 下位机在实车长距离直行时仍然持续左偏的问题，进行受控修正，使直行控制不再只依赖单一静态侧偏补偿，而具备实时闭环纠偏能力。

## Deliverable

- 下位机控制参数与必要逻辑调整，覆盖：
  - 对人工遥控源启用直线平衡回路
  - 调整直行纠偏触发阈值与补偿边界
  - IMU 有效时优先使用车体 Z 轴角速度作为直行偏航误差
  - IMU 无效时回退编码器估计的 `w_est`
  - 视需要对静态侧偏补偿做保守修正
- 编译级验证结果

## Constraints

- 不改变上位机协议
- 不改变底层电机驱动接口
- 不重做整套姿态融合
- 本轮优先做“可控、可回退”的闭环纠偏增强，不做大范围经验参数扫值

## Acceptance Criteria

- 直线平衡回路在 `PS2/ESP` 闭环直行场景下实际生效
- `PC` 控制源不叠加直线平衡，保留给航向保持
- 修正后代码仍可编译通过
- 改动保持边界清晰，便于继续实车微调

## Product Acceptance Criteria

- 长距离直行时，底盘不再只依赖固定 `STRAIGHT_SIDE_TRIM`
- 同一套参数对轻微机械不一致具备实时补偿能力
- 后续只需小范围实车调参，而不是再次改控制结构

## Manual Spot Checks

- 检查 `CTRL_USE_STRAIGHT_BALANCE` 是否启用
- 检查 `STRAIGHT_BALANCE_ALLOW_PS2/ESP` 与 `STRAIGHT_BALANCE_ALLOW_PC` 的分层开关
- 检查直行补偿回路触发条件是否合理
- 检查 IMU 有效和无效两条纠偏来源路径
- 检查静态 trim 是否仍保持保守范围

## Completion Language Policy

- 只有在代码修改完成且完成一轮编译验证后，才允许使用“已完成”
- 未经实车验证，不允许声称“左偏问题已最终消除”

## Delivery Truth Contract

- 本次交付只声称“完成了针对左偏的控制层修正”
- 不声称“已完成最终标定”，因为仍需要实车复测

## Non-Goals

- 不修改上位机导航参数
- 不做自动标定工具
- 不处理看门狗或故障锁存等其他问题

## Inferred Assumptions

- 当前底盘左偏属于轻中度持续偏航，而不是单轮明显打滑或硬件故障
- IMU 的车体 Z 轴角速度在校准有效时比编码器差速更适合抑制持续偏航
- 编码器速度仍可作为 IMU 无效时的后备直线平衡输入
