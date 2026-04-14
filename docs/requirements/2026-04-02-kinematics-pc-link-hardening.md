# Kinematics And PC Link Hardening Requirements

日期：2026-04-02

## Goal

在不改变上位机协议语义的前提下，修复 STM32 下位机的两类关键工程问题：

- 将底盘速度估计与速度指令映射从 `MAX_CPS` 归一化经验模型，改为显式差速底盘几何模型
- 将 `PC_Link` 的 ASCII 回复路径从阻塞串口发送，改为统一的非阻塞异步发送路径

## Deliverable

- 下位机代码修改，覆盖：
  - 显式轮径、轮距、编码器分辨率参数
  - 基于物理量纲的 `v_est / w_est / yaw_rate` 估计
  - 基于物理量纲的闭环目标到轮速参考映射
  - `PC_Link` ASCII 回复和流式遥测并入统一异步发送路径
- 必要的头文件接口更新
- 编译级验证结果

## Constraints

- 不修改上位机命令名称、消息格式、返回语义
- 不引入第二套独立 UART 发送状态机
- 不破坏已经完成的 `link_proto` 非阻塞发送行为
- 优先保证实时性、可维护性和量纲一致性

## Acceptance Criteria

- `robot_control.c` 中不再使用 `MAX_CPS` 直接推导 `v_est / w_est`
- 闭环 `v_ref / w_ref` 到左右轮参考值的映射改为基于差速底盘参数
- `PC_Link` 常规 ASCII 命令回复路径不再使用阻塞 `HAL_UART_Transmit`
- `PC_Link` 的流式遥测和 ASCII 回复共用统一的异步发送接口
- 工程可编译通过，或明确给出构建阻塞原因

## Product Acceptance Criteria

- 底盘状态估计与控制参考值具备一致的物理语义，便于后续标定和建图精度收敛
- PC 串口通道在命令回复和流式输出时不会阻塞主循环
- UART 发送忙时行为明确，不会卡死，不会破坏已有协议链路

## Manual Spot Checks

- 检查 `robot_config.h` 是否新增差速底盘物理参数
- 检查 `robot_control.c` 是否改为物理量纲建模
- 检查 `pc_link.c` 是否移除阻塞 `HAL_UART_Transmit`
- 检查 `HAL_UART_TxCpltCallback` 完成后是否能继续推进 PC 侧异步发送

## Completion Language Policy

- 只有在代码修改完成且完成一轮编译级验证后，才允许使用“已完成”
- 若只完成代码修改但未验证，必须明确说明

## Delivery Truth Contract

- 本次交付只声称“完成了运动学和 PC 串口发送路径的工程修正”
- 不声称“底盘精度已完成标定”或“建图精度已最终收敛”，除非有额外实测证据

## Non-Goals

- 不改上位机 ROS2 融合链
- 不引入完整串口总线仲裁框架
- 不处理看门狗条件喂狗、`Error_Handler()` 失效安全等其他剩余 P0 项

## Inferred Assumptions

- 当前底盘是标准差速结构，左右轮组可视作同侧等速
- 现有 `v/w` 命令语义表示归一化车体线速度和角速度请求
- `huart3` 上同时承载 PC ASCII 与 `LINK_ID_PC` 二进制协议
