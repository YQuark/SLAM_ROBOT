# 执行计划: ESP01S 与上位机协议语义对齐

## 计划

1. 检查 STM32 当前控制模式语义，确认 `SET_DRIVE` / `SET_RAW` 的边界
2. 修改 `ESP01S` 二进制桥接层，补上 `SET_RAW`
3. 修改 `ESP01S` HTTP 路由与网页控制逻辑，按当前模式发送正确命令类型
4. 补齐状态 JSON 中的控制接口提示字段
5. 更新协议文档，统一 binary、ASCII、HTTP 说明
6. 做静态核对，确认关键入口和错误提示已经一致

## 本轮结果

- 已补充 `MSG_CMD_SET_RAW = 0x13`
- 已新增 `GET /raw?l=&r=`
- 已限制 `GET /cmd` 仅闭环可用
- 已限制 binary `SET_DRIVE` 仅闭环可用
- 已更新协议文档
