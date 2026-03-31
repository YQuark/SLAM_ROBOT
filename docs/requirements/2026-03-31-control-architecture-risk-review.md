# 需求说明: 当前底盘控制与协议架构风险审查

## 目标

对当前底盘控制、状态上报、串口协议与 ESP01S 桥接进行一次静态一致性审查，找出仍会导致:

- 控制语义不一致
- 上位机误判状态
- 控制权切换发涩
- 手柄/上位机行为与预期不符

的问题点。

## 范围

- `Core/Src/Drivers/robot_control.c`
- `Core/Inc/Drivers/robot_control.h`
- `Core/Src/Drivers/link_proto.c`
- `Core/Src/Drivers/pc_link.c`
- `Core/Src/main.c`
- `ESP01S/ESP01S/ESP01S.ino`
- `docs/serial_protocols.md`

## 非目标

- 本轮不修改参数
- 本轮不做板上联调
- 本轮不调整 ROS/SLAM 代码
