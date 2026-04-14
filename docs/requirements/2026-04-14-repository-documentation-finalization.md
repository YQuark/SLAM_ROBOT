# Repository Documentation Finalization Requirements

日期：2026-04-14

## Goal

检查当前 STM32 底盘固件仓库的未提交代码、ESP-01S 桥接代码和既有文档，补齐与当前实现一致的文档说明，然后提交并通过 SSH 推送到 GitHub。

## Deliverable

- README 补充当前控制策略、IMU 校验和 ESP-01S 保活行为
- 串口协议文档补充控制源分层、IMU 静态校验和 HTTP 桥接时序契约
- 既有 2026-04-02 需求/计划文档修正为当前实现口径
- 本次治理运行的需求、计划和收据文件
- 构建或等效验证结果
- 使用标准提交前缀加中文说明提交并推送

## Constraints

- 不做新的业务逻辑改动
- 不覆盖已有未提交代码改动
- 不把本地工具运行日志当作本次文档交付内容
- GitHub 远端使用已有 SSH 配置
- 保留未实车复测的风险边界

## Acceptance Criteria

- 文档描述与当前 `robot_config.h`、`robot_control.c`、`mpu6050.c`、`ESP01S.ino` 行为一致
- 文档明确区分 PC、PS2、ESP 三类控制源的纠偏策略
- 文档明确 `imu_valid` 与 `imu_accel_valid` 的使用边界
- 文档明确 ESP 长按控制保活不应超过 STM32 `CMD_TIMEOUT_MS`
- 提交信息采用 `type: 中文说明` 格式
- 推送到 `origin/master` 成功

## Product Acceptance Criteria

- 后续上位机、ESP 调试和实车标定可以直接从文档理解当前接口契约
- 需求/计划记录能解释为什么当前策略不是单一航向保持或单一编码器直线补偿
- 未经实车验证的部分不会被文档写成最终结论

## Manual Spot Checks

- `README.md` 的控制架构和 ESP-01S 章节
- `docs/serial_protocols.md` 的状态字段、HTTP 桥接和 IMU 数据路径章节
- 4 月 2 日相关需求/计划文档是否仍存在与当前代码相反的描述
- `git status --short` 和最终推送结果

## Completion Language Policy

- 只有文档修改、验证、提交和推送都成功后，才允许说明“已提交并推送”
- 若构建失败或推送失败，必须说明具体失败点
- 未做实车复测，不声明“控制效果最终解决”

## Delivery Truth Contract

本次交付只声明“仓库当前代码与文档口径已经补齐并提交推送”。不声明底盘直行、IMU 姿态或 ESP 控制链路已经完成实车最终标定。

## Non-Goals

- 不重构 STM32 控制逻辑
- 不修改协议帧格式
- 不新增上位机代码
- 不清理与本次交付无关的历史跟踪文件

## Autonomy Mode

用户已明确要求检查、补文档、提交和推送；本次按自主执行处理，不在计划边界上额外暂停确认。

## Inferred Assumptions

- 当前未提交的 STM32/ESP 代码改动是用户希望纳入本次提交的有效阶段性成果
- `logs/combined.log` 和空 `.codex` 文件属于本地工具运行痕迹，不是本次文档交付的必要内容
