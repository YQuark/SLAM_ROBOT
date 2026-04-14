# UART Real-Time Execution Plan

日期：2026-04-02

## Internal Grade

`L`

原因：

- 改动集中在单一 STM32 工程
- 串口发送路径彼此耦合，拆成并行子任务容易引入接口冲突
- 需要串行完成：设计落锚、代码修改、回调接线、编译验证

## Steps

1. 识别运行期阻塞发送点，并区分运行期与初始化期
2. 设计并实现 `link_proto` 的非阻塞发送状态机
3. 接入 UART Tx complete 回调
4. 移除或禁用运行期主控制链中的阻塞打印
5. 运行编译级验证
6. 输出变更与风险说明

## Ownership Boundaries

- `Core/Src/Drivers/link_proto.c`
- `Core/Inc/Drivers/link_proto.h`
- `Core/Src/main.c`
- 如有必要：少量触达 `Core/Src/Drivers/mpu6050.c` 或 `Core/Src/Drivers/pc_link.c`

## Verification Commands

- 优先：`cmake --build /mnt/d/Document/Work/projects/Clion/build`
- 若构建目录不存在或失效：确认 `CMakeLists.txt` 与生成目录状态，再给出阻塞说明

## Delivery Acceptance Plan

- 代码路径改造完成
- 回调接线完整
- 编译命令执行过
- 对未覆盖的残余风险做说明

## Completion Language Rules

- 未做编译验证，不允许说“完成”
- 未处理的运行期阻塞点必须明确列出

## Rollback Rules

- 不改协议格式和公开消息语义
- 若异步发送引入明显接口破坏，则优先回退到“接口保持、内部重做”

## Phase Cleanup

- 留下 requirements / plan 文档
- 留下 runtime receipt
- 不保留临时脚本和测试垃圾文件
