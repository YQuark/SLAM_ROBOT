# Kinematics And PC Link Hardening Execution Plan

日期：2026-04-02

## Internal Grade

`L`

原因：

- 改动集中在单一 STM32 工程
- 运动学和 UART 发送路径都与现有实现强耦合，适合串行推进
- 需要先统一接口边界，再修改实现，再做编译验证

## Steps

1. 冻结差速底盘参数和发送路径收口边界
2. 将状态估计和闭环参考映射改为显式几何模型
3. 为 `PC_Link` 增加统一异步发送入口
4. 将 ASCII 回复和流式遥测接入该入口
5. 构建验证并记录剩余风险

## Ownership Boundaries

- `Core/Inc/Drivers/robot_config.h`
- `Core/Src/Drivers/robot_control.c`
- `Core/Inc/Drivers/link_proto.h`
- `Core/Src/Drivers/link_proto.c`
- `Core/Inc/Drivers/pc_link.h`
- `Core/Src/Drivers/pc_link.c`

## Verification Commands

- `cmake --build /mnt/d/Document/Work/projects/Clion/build/Debug`

## Delivery Acceptance Plan

- 差速底盘参数落地
- `robot_control.c` 量纲一致
- `pc_link.c` 不再存在命令回复阻塞发送
- 构建执行过并报告结果

## Completion Language Rules

- 未做编译验证，不允许说“完成”
- 若仍残留阻塞发送点，必须明确限定它们是否属于运行期关键路径

## Rollback Rules

- 不改变外部协议和命令语义
- 若异步发送改造影响 PC 端协议兼容性，优先回到“接口不变、内部缓冲收缩”的方案

## Phase Cleanup

- 留下 requirement / plan / runtime receipt
- 不新增临时脚本或测试垃圾文件
