# Repository Documentation Finalization Execution Plan

日期：2026-04-14

## Internal Grade

`L`

原因：

- 工作横跨代码审查、文档修正、验证、提交和推送
- 不需要多代理并行，关键路径是先确认当前实现，再串行修正文档
- 改动目标是文档收口，风险主要来自文档与代码行为不一致

## Wave Structure

1. Skeleton check
   - 确认仓库根目录、分支、远端、未提交变更和既有文档结构
2. Documentation alignment
   - 对照当前代码行为修正 README、协议文档和 4 月 2 日需求/计划
3. Verification
   - 检查关键术语引用
   - 执行可用构建命令
   - 查看最终 diff 和状态
4. Delivery
   - 选择性暂存本次相关代码、文档和运行收据
   - 使用标准前缀加中文提交说明提交
   - 通过 SSH 推送 `origin/master`

## Ownership Boundaries

- `README.md`
- `docs/serial_protocols.md`
- `docs/requirements/*.md`
- `docs/plans/*.md`
- `outputs/runtime/vibe-sessions/2026-04-14-documentation-finalization/`
- 仅允许对 `Core/Inc/Drivers/robot_config.h` 做注释级修正，不做功能改动

## Verification Commands

- `rg -n "YAW_HOLD_ALLOW|STRAIGHT_BALANCE_ALLOW|CONTROL_REFRESH_MS|CMD_IDLE_STOP_MS|imu_accel_valid|imu_valid" README.md docs Core/Inc/Drivers/robot_config.h ESP01S/ESP01S/ESP01S.ino`
- `cmake --build --preset Debug`
- `git diff --stat`
- `git status --short --branch`

## Delivery Acceptance Plan

- 文档已准确覆盖当前控制源分层：
  - PC 使用绝对航向保持
  - PS2/ESP 使用直行平衡，IMU 有效时以车体 Z 轴角速度为主
- 文档已准确覆盖 IMU 静态重力校验和 `imu_valid` / `imu_accel_valid` 边界
- 文档已准确覆盖 ESP 保活周期和 HTTP 去重刷新行为
- 构建结果和推送结果明确记录

## Completion Language Rules

- 未通过构建或未完成推送，不允许使用“已提交并推送”
- 若本地工具日志未纳入提交，需要在最终说明中明确其处理方式
- 未实车复测的控制效果必须以风险或待验证项描述

## Rollback Rules

- 若文档修正方向与代码事实冲突，优先回到代码事实，不按旧需求文档描述
- 若构建失败，不继续推送，先报告阻塞
- 不回退用户已有业务代码改动

## Phase Cleanup Expectations

- 留下 skeleton、intent、plan-execute 和 cleanup 收据
- 不保留临时脚本
- 不新增与本次交付无关的生成文件
