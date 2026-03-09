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

## 8. 注意事项

- 本仓库可能包含本地 IDE 配置与临时构建目录，请按需忽略后再提交。
- 若重新生成 CubeMX 代码，请重点回归测试：
  - GPIO 复用（编码器/PS2 引脚）
  - 定时器模式（PWM/编码器）
  - 串口与 I2C 初始化顺序

