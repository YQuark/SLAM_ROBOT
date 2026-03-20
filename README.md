# SLAM_ROBOT（STM32F407）

基于 STM32F407 的四轮机器人控制项目，包含电机驱动、编码器测速、IMU 采集、电池保护、OLED 显示，以及多来源控制（PS2 手柄 / ESP-01S / PC 串口）。

## 1. 主要功能

- 四路电机控制（开环/闭环）
- 编码器速度反馈与 PI 控制
- MPU6050 姿态数据读取
- 电池电压检测与欠压保护
- SSD1306 OLED 状态显示
- 控制源仲裁：PS2、ESP、PC
- 看门狗与基础自检机制

## 2. 目录说明

- `Core/Src/`：主逻辑与外设初始化代码
- `Core/Src/Drivers/`：项目驱动与控制模块（`motor`/`encoder`/`ps2`/`robot_control` 等）
- `Core/Inc/Drivers/`：驱动头文件与核心配置（`robot_config.h`）
- `Drivers/`：STM32 HAL/CMSIS 库
- `cmake/`：CMake 与交叉编译工具链配置
- `ESP01S/`：ESP-01S 相关资料
- `【WHEELTEC】PS2遥控手柄附送资料/`：PS2 参考资料与例程

## 3. 开发环境

- STM32CubeCLT（含 `arm-none-eabi-gcc`、`ninja`）
- CMake >= 3.22
- 调试/下载工具：ST-Link（或等效下载器）

建议在 CLion 中直接打开工程，使用项目内 CMake 配置。

## 4. 编译

### 4.1 命令行构建

```bash
cmake --preset Debug
cmake --build --preset Debug
```

构建产物位于：

- `build/Debug/Clion.elf`
- `build/Debug/Clion.map`

### 4.2 烧录

可使用 STM32CubeProgrammer / ST-Link 工具将 `Clion.elf`（或转换后的 `.bin/.hex`）烧录到目标板。

## 5. 运行与调试

- 主调试串口：`USART1`
- 其余链路：`USART2`（ESP）、`USART3`（PC）
- 默认会输出初始化日志、控制状态和遥测信息（详见 `robot_config.h`）
- 串口/链路协议详表：`docs/serial_protocols.md`

### 5.1 通信架构总览

本仓库内固件当前包含 3 路串口通信：

- `USART1`：本地调试日志与 FireWater 数值遥测输出
- `USART2`：与 `ESP-01S` 通信，使用二进制帧协议
- `USART3`：与 PC/上位机通信，同时支持二进制帧协议和 ASCII 控制台

说明：

- ROS/SLAM/导航相关上位机程序属于独立项目，不包含在本仓库内
- 本仓库只定义 STM32 固件侧串口协议、状态字段和调试接口

### 5.2 串口参数

除特殊说明外，3 路 UART 默认参数一致：

- 波特率：`115200`
- 数据位：`8`
- 校验位：`None`
- 停止位：`1`
- 流控：`None`

### 5.3 USART1：调试日志与 FireWater 遥测

用途：

- 上电启动日志
- 外设初始化结果
- IMU 校准与异常输出
- 周期性 CSV 遥测

典型日志包括：

- 复位原因
- I2C 扫描结果
- `MPU6050 Init OK`
- `IMU calibration completed`
- `IMU calibration FAILED`
- `PS2 Init done`

当 `TELEMETRY_ENABLE=1` 时，`USART1` 会输出纯数字 CSV，可直接接 VOFA+ FireWater。

当前字段顺序为：

1. `tick_ms`
2. `imu_valid`
3. `ax_mg`
4. `ay_mg`
5. `az_mg`
6. `gx_cdps`
7. `gy_cdps`
8. `gz_cdps`
9. `enc0_cps`
10. `enc1_cps`
11. `enc2_cps`
12. `enc3_cps`
13. `v_cmd_x1000`
14. `w_cmd_x1000`
15. `u0_x1000`
16. `u1_x1000`
17. `u2_x1000`
18. `u3_x1000`
19. `roll_x100_deg`
20. `pitch_x100_deg`
21. `yaw_x100_deg`
22. `battery_mv`
23. `battery_pct_x10`
24. `enc_fault_mask`

### 5.4 USART2 / USART3：二进制帧协议

`USART2` 和 `USART3` 共用同一套二进制协议，适用于上位机程序或 ESP 侧程序。

外层封装：

- 传输编码：`COBS`
- 帧尾分隔：单字节 `0x00`

解码后帧格式：

1. `SOF` = `0xA5`
2. `VER` = `0x01`
3. `MSG_TYPE`
4. `FLAGS`
5. `SEQ`
6. `PAYLOAD_LEN`，小端
7. `PAYLOAD`
8. `CRC16`，小端，`CRC-16/CCITT-FALSE`
9. `EOF` = `0x5A`

常用消息类型：

- `0x01`：`PING`
- `0x02`：`GET_STATUS`
- `0x10`：`SET_DRIVE`
- `0x11`：`SET_MODE`
- `0x12`：`SET_IMU`
- `0x30`：`GET_DIAG`
- `0x31`：`CLEAR_DIAG`
- `0x32`：`GET_FAULT_LOG`
- `0x33`：`CLEAR_FAULT_LOG`

`GET_STATUS` ACK 当前包含这些核心字段：

- `tick_ms`
- `mode`
- `src`
- `battery_mv`
- `battery_pct_x10`
- `v_cmd_q15`
- `w_cmd_q15`
- `gz_dps_x10`
- `imu_enabled`
- `imu_valid`
- `v_est_q15`
- `w_est_q15`
- `enc_fault_mask`
- `enc_vel_cps[4]`

其中：

- `v_cmd_q15` / `w_cmd_q15` 为归一化控制量，范围约 `[-32767, 32767]`
- `v_est_q15` / `w_est_q15` 为固件侧估计速度反馈
- `enc_fault_mask` 用于表示编码器异常状态位

### 5.5 USART3：ASCII 控制台

`USART3` 额外支持面向人工调试的 ASCII 命令行，适合 Windows 串口助手直接验证。

当前支持命令：

- `PING`
- `HELP`
- `STATUS`
- `DEBUG`
- `MODE <0|1|2>`
- `DRIVE <v> <w>`
- `CTRL <v> <w>`
- `MOTOR <left> <right>`
- `STOP`
- `IMU <0|1>`
- `STREAM <ON|OFF> [hz]`
- `RATE <hz>`

典型用途：

- `STATUS`：查看模式、控制源、电池、IMU 状态
- `DEBUG`：查看参考速度、实测速度、控制输出、PWM 寄存器和故障位
- `DRIVE`：发送底盘线速度/角速度命令
- `MOTOR`：直接左右轮输出，适合定位“通信问题”还是“电机执行问题”
- `STREAM ON 10`：按设定频率持续输出 `TEL,...` 状态行

当前 `STATUS` 回包格式：

```text
OK STATUS mode=<u> src=<u> v_x1000=<i> w_x1000=<i> vbat_mv=<i> imu_en=<u> imu_ok=<u> gz_cdps=<i> motor_ovr=<u>
```

当前 `DEBUG` 回包格式：

```text
OK DEBUG mode=<u> src=<u> ref=<i>,<i>,<i>,<i> meas=<i>,<i>,<i>,<i> u_x1000=<i>,<i>,<i>,<i> ccr=<u>,<u>,<u>,<u> dir=<u>,<u>,<u>,<u> fault=0xNN
```

`STREAM ON` 时输出的 `TEL` 行为定点整数 CSV，便于串口助手或脚本直接解析。

关键配置文件：

- `Core/Inc/Drivers/robot_config.h`

可在其中调整：

- 调度周期（PS2/控制/IMU/电池/OLED）
- 控制参数（PI、限幅、斜坡、启动助推）
- 安全参数（欠压阈值、看门狗）
- 调试开关（`TELEMETRY_ENABLE`、`CTRL_DEBUG_ENABLE` 等）

## 6. PS2 手柄接入说明（当前工程）

### 6.1 引脚定义

以 `Core/Inc/main.h` 中定义为准：

- `PS2_CLK` -> `PA5`
- `PS2_DAT` -> `PA6`
- `PS2_CMD` -> `PA7`
- `PS2_CS`  -> `PC4`
- 电源：`3.3V~5V`
- 地：`GND`

### 6.2 已实现特性

- 参考 WHEELTEC 例程完成初始化序列（Short Poll + 配置模式）
- 自动检测 CMD/DAT 互换接线（接反可自动适配）
- 扫描失败时自动重配置
- 支持摇杆与方向键后备控制

若串口出现 `PS2: Invalid mode`，优先检查：

- 接线顺序与供电
- 手柄接收器与主板共地
- `PS2_ENABLE` 是否开启（`robot_config.h`）

## 7. 常用源码入口

- 主循环：`Core/Src/main.c`
- 控制器：`Core/Src/Drivers/robot_control.c`
- 电机：`Core/Src/Drivers/motor.c`
- 编码器：`Core/Src/Drivers/encoder.c`
- PS2：`Core/Src/Drivers/ps2.c`
- IMU：`Core/Src/Drivers/mpu6050.c`
- 协议文档：`docs/serial_protocols.md`

## 8. 注意事项

- 本仓库可能包含本地 IDE 配置与临时构建目录，请按需忽略后再提交。
- 若重新生成 CubeMX 代码，请重点回归测试：
  - GPIO 复用（编码器/PS2 引脚）
  - 定时器模式（PWM/编码器）
  - 串口与 I2C 初始化顺序
