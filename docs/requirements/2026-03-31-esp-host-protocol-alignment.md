# 需求说明: ESP01S 与上位机协议语义对齐

## 背景

当前底盘控制已经将 `MODE_OPEN_LOOP` 与 `MODE_CLOSED_LOOP` 的语义分离:

- 开环只允许显式原始左右输出
- 闭环只允许底盘速度命令

但 `ESP01S` HTTP 控制面和协议文档仍残留旧语义，容易出现:

- 开环模式下仍然发送 `SET_DRIVE`
- 上位机文档与固件行为不一致
- 二进制、ASCII、HTTP 三套接口各说各话

## 目标

1. `ESP01S` 在开环模式发送 `SET_RAW`
2. `ESP01S` 在闭环模式发送 `SET_DRIVE`
3. `ESP01S` 网页端明确展示当前控制接口语义
4. 固件二进制、ASCII、HTTP 协议文档统一成一版说明

## 非目标

- 不在这轮调整底盘 PI 参数
- 不在这轮重构上位机程序本体
- 不在这轮改 ROS/SLAM 接口

## 验收标准

1. `ESP01S` 增加显式 `SET_RAW(0x13)` 发送能力
2. `GET /cmd` 只允许闭环
3. `GET /raw` 只允许开环
4. Binary `SET_DRIVE` 与 ASCII `DRIVE` 一样对模式做拒绝
5. `docs/serial_protocols.md` 同时覆盖 binary、ASCII、ESP HTTP 三套语义
