# STM32 Audit Remediation Execution Plan

Date: `2026-03-25`
Run: `20260325-stm32-audit-remediation`
Internal grade: `L`

## Why L

这不是简单单文件修补。需要先核对审计结论是否仍成立，再对中断、控制节拍和安全状态机做跨模块修改，并在不破坏现有串口协议的前提下完成验证。

补充原因：

- 本轮不再把“尽量降级功能”作为默认整改方向，而是要先按整机链路重新判断问题优先级。
- 该项目的产品目标依赖上位机导航建图链路连续性，因此计划必须围绕“控制执行 + 状态更新 + 状态回传 + 指令仲裁”的关键路径展开。

## Wave Structure

### Wave 1: System-Level Audit Reframe

Owner: main agent

- 重新核对关键链路与非关键链路
- 识别会直接破坏上位机导航/建图连续性的系统级问题
- 区分“局部代码问题”和“整机运行语义问题”

### Wave 2: Arbitration and State-Flow Audit

Owner: main agent

- 复核 `PC / PS2 / ESP` 仲裁语义
- 核对“链路在线”“控制权”“运动命令有效”“状态查询活动”之间是否混用
- 明确哪些链路语义会影响关键控制行为

### Wave 3: Critical Path Pressure Audit

Owner: main agent

- 核对主循环中哪些工作会挤占控制执行、状态更新和状态回传
- 区分“必须重构的问题”和“可保留但需让路的问题”
- 重点看串口、I2C、调试输出和本地显示对关键链路的影响

### Wave 4: Productization Backlog Reorder

Owner: main agent

- 重新给出产品化问题优先级
- 避免把“安全自保”误写成“功能粗暴降级”
- 将整改项分成关键链路类、行为语义类、调试隔离类

### Wave 5: Verification and Cleanup

Owner: main agent

- 更新需求与计划文档
- 保留现有代码构建证据作为本轮静态审计基础
- 核对关键行为路径
- 输出 phase artifact 和 cleanup receipt

## Ownership Boundaries

- `Core/Src/main.c`: 主循环关键路径、控制执行、状态回传、调试路径
- `Core/Src/Drivers/robot_control.c`: 多源指令仲裁、状态语义、低速控制行为
- `Core/Src/Drivers/pc_link.c`: PC ASCII 控制、调试旁路、链路活动语义
- `Core/Src/Drivers/link_proto.c`: 二进制协议、状态查询、ACK/NACK 和链路活动语义
- `Core/Src/Drivers/esp_link.c`: ESP 链路 ISR 行为
- `Core/Src/Drivers/mpu6050.c`: 姿态更新阻塞与观测链路影响
- `Core/Src/Drivers/ssd1306.c`: 本地显示对关键链路的影响
- `Core/Inc/Drivers/*.h`: 配置和语义边界定义

## Verification Commands

```bash
cmake --build --preset Debug
rg -n "choose_cmd|RobotControl_NotifyLinkActivity|GET_STATUS|HAL_UART_Transmit|MPU6050_ReadRaw|SSD1306_UpdateScreen|ESP_Link_UART_RxCpltCallback|LinkProto_RxByte|MOTOR" Core/Src Core/Inc
```

## Rollback Rules

- 不为了追求局部安全而直接取消上位机依赖的关键观测或回传能力。
- 不把“非关键任务让路”误实现为“功能永久关闭”。
- 不修改串口协议字段含义，除非有明确必要并同步文档。
- 若发现现有整改把“状态轮询”错误地提升成“控制接管”语义，需要优先修正语义边界而不是继续堆保护逻辑。

## Prioritized Backlog

### P0: 直接影响导航建图连续性的项

1. 明确并拆分链路语义
- 把“链路在线”“控制权持有”“运动命令有效”“状态轮询活跃”拆成独立状态。
- 修正当前 `GET_STATUS/STATUS` 这类查询流量会刷新 `PC` 活跃状态并间接影响控制仲裁的问题。
- 相关文件：
  - `Core/Src/Drivers/robot_control.c`
  - `Core/Src/Drivers/link_proto.c`
  - `Core/Src/Drivers/pc_link.c`

2. 给关键链路建立可观测性
- 增加控制周期积压/抖动计数。
- 增加状态回传“年龄”或最近一次刷新时间戳。
- 增加命令源切换计数与 UART 错误恢复计数。
- 目标是让“呼吸灯变慢、静止抽动、链路抢占”这类问题能被代码证据直接定位。

3. 关键链路让路规则显式化
- 不是一刀切地去掉 `OLED/DEBUG/TEL`，而是明确它们在系统忙时如何限频、跳过或让路。
- 保证控制执行、编码器/姿态更新、上位机命令接入、关键状态回传优先于附属任务。
- 相关文件：
  - `Core/Src/main.c`
  - `Core/Src/Drivers/mpu6050.c`
  - `Core/Src/Drivers/ssd1306.c`

4. 收口调试旁路对产品路径的影响
- `MOTOR` 直驱能力必须受超时、故障态或编译开关约束，不能无限期维持输出。
- 调试串口输出必须可统一关闭或限频，不能在关键路径上无限阻塞。
- 相关文件：
  - `Core/Src/Drivers/pc_link.c`
  - `Core/Src/main.c`
  - `Core/Src/Drivers/*.c`

### P1: 影响稳定性，但可在 P0 后处理

1. 统一两条外部链路的接收策略
- `USART2/ESP` 路径也改成“ISR 只入队，poll 做解析”。

2. 强化 UART 错误恢复
- 从“清 flag + 继续收”升级为“连续错误计数 + 节流 + 必要时重置外设”。

3. 统一整机故障矩阵
- 定义编码器、IMU、链路、欠压、通信错误分别属于“继续运行 / 受限运行 / 必须停机”哪一类。

### P2: 行为优化和可维护性项

1. 重构低速控制为显式状态机
- 把零速、起步、低速跟踪、正常跟踪、故障停机拆成可解释状态。

2. 整理驱动层日志出口
- 驱动里不再直接散落 `HAL_UART_Transmit()`。

3. 改造 `Error_Handler()`
- 进入安全输出态并留下可诊断证据。

## Phase Cleanup Expectations

- 写入 `phase-*.json` 记录
- 写入 `cleanup-receipt.json`
- 不保留临时调试文件
