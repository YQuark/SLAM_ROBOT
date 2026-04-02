# SLAM_ROBOT（STM32F407）

基于 `STM32F407` 的四轮差速底盘控制工程。仓库包含 STM32 固件、串口协议、`ESP-01S` 桥接代码和协议文档，覆盖电机驱动、编码器测速、姿态采集、电池保护、OLED 显示，以及 `PS2 / ESP / PC` 三路控制输入。

当前仓库边界很明确：

- 本仓库负责底盘固件、底盘状态字段和板载通信协议
- ROS / SLAM / 导航 / 地图构建等上位机系统不在当前仓库内
- 上位机应把这里定义的状态字段和控制语义当作接口契约使用

更完整的协议细节见 [docs/serial_protocols.md](/mnt/d/Document/Work/projects/Clion/docs/serial_protocols.md)。

## 当前能力

- 四路电机控制，支持 `MODE_IDLE / MODE_OPEN_LOOP / MODE_CLOSED_LOOP`
- 四路编码器测速、滤波、静止校准和故障检测
- `MPU6050` 采集、静态校准和姿态融合
- 电池电压检测与欠压保护
- `SSD1306` OLED 状态显示
- 多来源控制仲裁：`PS2`、`ESP-01S`、`PC`
- `USART1 / USART2 / USART3` 三路串口分工
- 二进制链路协议 + `USART3` ASCII 调试控制台

## 仓库结构

- `Core/Src/`：主循环、调度、外设初始化
- `Core/Src/Drivers/`：项目驱动与控制核心
- `Core/Inc/Drivers/`：驱动头文件、运行参数和协议定义
- `Drivers/`：STM32 HAL / CMSIS
- `ESP01S/`：ESP-01S 桥接固件
- `docs/`：协议说明、需求和计划文档
- `cmake/`：交叉编译工具链配置

主要入口文件：

- `Core/Src/main.c`
- `Core/Src/Drivers/robot_control.c`
- `Core/Src/Drivers/motor.c`
- `Core/Src/Drivers/encoder.c`
- `Core/Src/Drivers/mpu6050.c`
- `Core/Src/Drivers/pc_link.c`
- `Core/Src/Drivers/link_proto.c`
- `ESP01S/ESP01S/ESP01S.ino`

## 构建与烧录

依赖环境：

- `STM32CubeCLT`（含 `arm-none-eabi-gcc`、`ninja`）
- `CMake >= 3.22`
- `ST-Link` / `STM32CubeProgrammer` 或等效下载工具

命令行构建：

```bash
cmake --preset Debug
cmake --build --preset Debug
```

典型产物：

- `build/Debug/Clion.elf`
- `build/Debug/Clion.map`

烧录时使用 `Clion.elf` 或转换后的 `.bin/.hex` 文件即可。

## 控制架构

### 运行模式

- `MODE_IDLE = 0`
  - 安全怠速
  - 电机停止输出
- `MODE_OPEN_LOOP = 1`
  - 原始左右侧输出模式
  - 只接受显式 `left/right raw` 命令
- `MODE_CLOSED_LOOP = 2`
  - 正常速度闭环模式
  - 只接受归一化 `v/w` 命令

当前模式策略：

- 上电默认进入 `MODE_CLOSED_LOOP`
- `PS2` 从 `MODE_IDLE` 起控时会自动进入 `MODE_CLOSED_LOOP`
- 开环必须显式切到 `MODE_OPEN_LOOP`

### 指令来源

- `CMD_SRC_PS2`
- `CMD_SRC_ESP`
- `CMD_SRC_PC`

控制器会按命令时效和来源仲裁，状态里会回报当前 `src`、链路在线状态、控制状态和命令语义。

### 命令语义

当前代码已经把“速度命令”和“原始输出命令”彻底分开：

- `cmd_semantics = 0`：无命令
- `cmd_semantics = 1`：闭环速度命令 `v/w`
- `cmd_semantics = 2`：开环原始输出 `raw_left/raw_right`

这也是上位机、ESP 桥接和串口调试时应该遵守的语义边界。

## 串口与协议

默认串口参数：

- 波特率：`115200`
- 数据位：`8`
- 校验位：`None`
- 停止位：`1`
- 流控：`None`

### USART1

用途：

- 本地调试日志
- 启动信息
- IMU 校准日志
- FireWater / VOFA+ 风格 CSV 遥测

说明：

- `USART1` 是调试流，不是上位机正式 odom 契约
- 若 `TELEMETRY_ENABLE=1`，会输出固定字段顺序的 CSV 遥测

### USART2

用途：

- `ESP-01S` 桥接链路

协议：

- 只使用二进制帧协议

### USART3

用途：

- PC / 上位机链路

协议：

- 二进制帧协议
- ASCII 控制台

### 二进制帧协议

`USART2` 和 `USART3` 共用同一套协议：

- 传输编码：`COBS`
- 帧分隔：尾部 `0x00`
- `SOF = 0xA5`
- `EOF = 0x5A`
- `CRC = CRC-16/CCITT-FALSE`

常用消息类型：

- `0x01` `PING`
- `0x02` `GET_STATUS`
- `0x10` `SET_DRIVE`
- `0x11` `SET_MODE`
- `0x12` `SET_IMU`
- `0x13` `SET_RAW`

语义约束：

- `SET_DRIVE` 只允许在 `MODE_CLOSED_LOOP`
- `SET_RAW` 只允许在 `MODE_OPEN_LOOP`
- 模式不匹配时会返回 `NACK`

`GET_STATUS` 当前会返回这些核心信息：

- `tick_ms`
- `mode`
- `src`
- `cmd_semantics`
- `v_cmd_q15`
- `w_cmd_q15`
- `raw_left_q15`
- `raw_right_q15`
- `v_est_q15`
- `w_est_q15`
- `yaw_est_x100_deg`
- `gz_dps_x10`
- `imu_enabled`
- `imu_valid`
- `imu_accel_valid`
- `enc_fault_mask`
- `enc_vel_cps[4]`
- `battery_mv`
- `battery_pct_x10`

### 当前上位机对接建议

如果上位机使用 `Ros2_Slam/src/stm32_robot_bridge`，当前应按下面这条契约理解下位机数据：

- 平移速度来源：`v_est_q15`
- 角速度来源：`w_est_q15`
- 航向来源：`yaw_est_x100_deg`
- 陀螺辅助：`gz_dps_x10`
- IMU 置信标志：`imu_valid`
- 加速度置信标志：`imu_accel_valid`
- 轮速诊断：`enc_vel_cps[4]`
- 故障诊断：`enc_fault_mask`

对接边界要点：

- `v_est / w_est` 是下位机基于底盘几何参数和编码器速度估计出的底盘运动量，适合做底盘反馈和 odom 速度输入。
- `yaw_est` 是下位机当前用于控制与状态回报的融合航向，优先级高于上位机自己积分原始 `gz`。
- `raw_accel_mg[3]` 和 ASCII `TEL` 里的原始加速度目前只用于调试，不应用于上位机平移积分。
- `imu_valid=0` 时，上位机不应把 `yaw_est` 当成高可信 IMU 航向使用。
- `enc_fault_mask != 0` 时，上位机应把底盘里程计视为异常状态，并触发诊断或安全处理。

### USART3 ASCII 控制台

支持命令：

- `PING`
- `HELP`
- `STATUS`
- `DEBUG`
- `MODE <0|1|2>`
- `DRIVE <v> <w>`
- `CTRL <v> <w>`
- `RAW <left> <right>`
- `MOTOR <left> <right>`
- `STOP`
- `IMU <0|1>`
- `STREAM <ON|OFF> [hz]`
- `RATE <hz>`

使用约束：

- `DRIVE/CTRL` 只用于闭环速度控制
- `RAW/MOTOR` 只用于开环原始输出
- `STREAM ON` 会持续输出 `TEL,...` 状态流，便于上位机或串口工具消费

`STATUS` 重点字段：

- `mode`
- `src`
- `cmd_sem`
- `v_cmd_x1000`
- `w_cmd_x1000`
- `raw_l_x1000`
- `raw_r_x1000`
- `v_est_x1000`
- `w_est_x1000`
- `yaw_x100`
- `imu_en`
- `imu_ok`
- `imu_acc_ok`
- `enc_fault`

ASCII 状态流和正式对接的关系：

- `STATUS` 和 `TEL` 主要用于串口调试、人工观察和快速校验字段含义。
- 正式上下位机衔接优先使用 `GET_STATUS` 二进制状态包，而不是长期依赖 ASCII `TEL`。
- 如果临时用 ASCII 做调试桥接，至少应取 `v_est_x1000`、`w_est_x1000`、`yaw_x100`、`gz_cdps`、`imu_ok`、`imu_acc_ok`、`enc_fault` 这些字段，不要自行猜测列顺序。

## ESP-01S 桥接

`ESP01S/ESP01S/ESP01S.ino` 当前实现的是一个 HTTP 到串口协议的轻量桥接。

主要端点：

- `GET /mode?m=<0|1|2>`
- `GET /cmd?v=<float>&w=<float>`
- `GET /raw?l=<float>&r=<float>`
- `GET /status`
- `GET /health`

当前约束：

- `/cmd` 只允许在闭环模式
- `/raw` 只允许在开环模式
- `/mode` 会等待 STM32 ACK 后再返回成功

`/status` 返回的关键字段：

- `mode`
- `mode_name`
- `control_api`
- `cmd_space`
- `src`
- `v`
- `w`
- `raw_left`
- `raw_right`
- `imu`
- `enc`

说明：

- `control_api` 表示当前应该调用哪类控制接口：`none / raw / drive`
- `cmd_space` 表示最近一帧 STM32 状态里的命令语义：`none / velocity / raw`

## Odom 与 IMU 使用边界

当前 odom 策略是“编码器主导，IMU 航向辅助”。

推荐给上位机使用的字段：

- `v_est`
- `w_est`
- `yaw_est`
- bias-corrected `gz`
- 四路编码器速度
- `enc_fault_mask`
- `imu_valid`
- `imu_accel_valid`

当前策略：

- 平移里程以编码器为主
- 航向可参考 `yaw_est`
- `raw_accel` 仍然只用于调试，不应直接参与平移积分

注意：

- 当前 `imu_valid=1` 只表示“航向/陀螺辅助可用”
- `imu_accel_valid=1` 才表示加速度计当前通过静态校准和运行期门限
- 当前 IMU 没有磁力计绝对航向参考，长距离直行时仍可能存在航向漂移，需要结合编码器和上位机侧进一步处理

## PS2 手柄

当前工程内 PS2 采用 GPIO 软件时序方式接入。

默认引脚：

- `PS2_CLK -> PA5`
- `PS2_DAT -> PA6`
- `PS2_CMD -> PA7`
- `PS2_CS  -> PC4`

已实现能力：

- 初始化序列
- CMD/DAT 互换自动适配
- 异常后重配置
- 摇杆与方向键后备控制

如果出现 `PS2: Invalid mode`，优先检查：

- 供电与共地
- 接线顺序
- 手柄接收器状态
- `PS2_ENABLE` 是否开启

## 关键配置

主要运行参数在 `Core/Inc/Drivers/robot_config.h`：

- 调度周期
- PI 参数
- 航向融合与直行补偿参数
- 欠压保护参数
- 看门狗参数
- 调试输出开关

调控制参数时，优先区分这几类问题：

- 固定偏一侧：先看底盘机械和静态侧边配平
- 长距离慢慢漂：先看航向估计和 IMU 使用方式
- 随机大波动：先看 IMU、编码器、供电和串口实时性

## 常用调试入口

- `Core/Src/main.c`
- `Core/Src/Drivers/robot_control.c`
- `Core/Src/Drivers/motor.c`
- `Core/Src/Drivers/encoder.c`
- `Core/Src/Drivers/mpu6050.c`
- `Core/Src/Drivers/pc_link.c`
- `Core/Src/Drivers/link_proto.c`
- [docs/serial_protocols.md](/mnt/d/Document/Work/projects/Clion/docs/serial_protocols.md)

## 当前已知边界

- 当前 README 以“当前代码已实现什么、怎么用”为主，不替代详细协议文档
- FireWater / `USART1` 遥测是调试流，不是正式上位机协议
- 当前 IMU 姿态链路没有磁力计绝对航向约束，长距离航向稳定性仍需结合编码器与上位机策略验证
- 若重新生成 CubeMX 代码，需重点回归：
  - GPIO 复用
  - 编码器 / PWM 定时器模式
  - UART / I2C 初始化顺序
  - IMU 启动与校准流程
- 协议文档：`docs/serial_protocols.md`

## 8. 注意事项

- 本仓库可能包含本地 IDE 配置与临时构建目录，请按需忽略后再提交。
- 若重新生成 CubeMX 代码，请重点回归测试：
  - GPIO 复用（编码器/PS2 引脚）
  - 定时器模式（PWM/编码器）
  - 串口与 I2C 初始化顺序
