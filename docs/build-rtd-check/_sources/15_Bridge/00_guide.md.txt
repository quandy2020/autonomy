# Bridge 通信桥接指南

`autonomy/bridge` 是 Autonomy 的**外部通信桥接**子系统，在机器人本地导航栈与外部客户端（移动端、云平台、调度系统）之间提供统一 RPC / 消息通道。本文档为模块总入口，以下按 **§1–§8** 顺序组织。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 新手 | [§1 概览](01_overview.md) → [§2 快速开始](02_quickstart.md) → [§5 架构](05_architecture.md) → [§4 使用指南](04_usage.md) |
| 协议研发 | [§8 综述](08_survey.md) → [§3 数学原理](03_math.md) → [§6 gRPC](06_grpc.md) / [§7 MQTT](07_mqtt.md) |
| 集成调试 | [§2 快速开始](02_quickstart.md) → [§4 使用指南](04_usage.md) → [§6 gRPC API](06_grpc.md#64-autonomyservice-接口) → [§4.7 排错](04_usage.md#47-故障排查) |

| 编号 | 文件 | 内容 |
|------|------|------|
| 1 | [01_overview.md](01_overview.md) | 模块概览 |
| 2 | [02_quickstart.md](02_quickstart.md) | 快速开始 |
| 3 | [03_math.md](03_math.md) | 数学原理 |
| 4 | [04_usage.md](04_usage.md) | 使用指南 |
| 5 | [05_architecture.md](05_architecture.md) | 模块架构设计 |
| 6 | [06_grpc.md](06_grpc.md) | gRPC 桥接插件 |
| 7 | [07_mqtt.md](07_mqtt.md) | MQTT 桥接插件 |
| 8 | [08_survey.md](08_survey.md) | 通信协议综述 |
