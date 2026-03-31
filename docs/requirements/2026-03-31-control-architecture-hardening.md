# 需求说明: 底盘控制架构硬化与协议单义化

## 目标

将当前底盘控制、状态上报与 ESP01S 桥接中剩余的架构问题一次收口，重点解决:

- PS2 自动进入错误模式
- PC 控制权释放不及时
- 运行时状态字段同名双义
- ESP01S 状态/UI 与底盘真实命令语义不一致

## 验收标准

1. PS2 从 `MODE_IDLE` 自动唤醒时进入 `MODE_CLOSED_LOOP`
2. PC 控制锁定必须依赖“仍然有效的 PC 命令”，不能只靠旧控制痕迹
3. 状态上报必须显式区分 `velocity` 与 `raw_left_right`
4. Binary / ASCII / ESP HTTP 文档与代码保持一致
5. ESP01S 模式切换必须等待 STM32 ACK
