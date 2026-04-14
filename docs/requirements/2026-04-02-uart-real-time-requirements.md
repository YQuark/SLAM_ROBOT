# UART Real-Time Requirements

日期：2026-04-02

## Goal

在不改变现有上下位机协议语义的前提下，修复 STM32 下位机运行期串口发送对控制周期的阻塞影响。

## Deliverable

- 下位机代码修改，覆盖：
  - 运行期控制链路中的阻塞串口输出清理
  - `link_proto` 二进制协议发送改为非阻塞异步发送
- 必要的回调接线更新
- 编译级验证结果

## Constraints

- 不修改协议帧格式、消息类型、ACK/NACK 语义
- 不改变上位机接口
- 优先保证运行期实时性，而不是保留所有调试输出能力
- 初始化/自检阶段的阻塞打印可暂时保留

## Acceptance Criteria

- 控制闭环主路径中不再出现 `HAL_UART_Transmit(..., 100)` 这类运行期阻塞发送
- `LinkProto_Poll()` 不再直接阻塞发送 UART 帧
- 协议发送通过中断完成回调推进，不在主循环等待
- 编译通过或给出明确编译阻塞原因

## Product Acceptance Criteria

- 在运行期，串口发送不会再直接挤压控制周期
- 当链路发送忙时，协议层行为明确且可恢复，不会卡死主循环
- 代码变更保持可维护，接口边界清晰

## Manual Spot Checks

- 检查 `main.c` 运行期循环中的阻塞打印是否已移除
- 检查 `link_proto.c` 是否改为异步发送状态机
- 检查 `HAL_UART_TxCpltCallback` 是否能正确驱动协议发送完成

## Completion Language Policy

- 只有在代码修改完成且至少完成一轮编译级验证后，才允许使用“已完成”
- 若未完成编译验证，必须明确说明

## Delivery Truth Contract

- 本次交付仅声称“实现了代码级非阻塞改造”
- 不声称“实时性问题完全解决”或“无回归”，除非有额外实测证据

## Non-Goals

- 不重构整套 UART 抽象层
- 不统一 ASCII 调试和二进制协议到一个完整总线仲裁器
- 不修改看门狗、故障锁存等其他 P0 项

## Inferred Assumptions

- `huart1` 同时承担 PC ASCII 调试与 PC 二进制协议
- `huart2` 或其他链路通过 `link_proto` 复用同类发送机制
- 现阶段最优策略是“运行期实时链先脱阻塞”，不是一次性做完整通信框架重构
