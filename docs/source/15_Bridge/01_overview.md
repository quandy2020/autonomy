(bridge-overview)=
# 1. 模块概览

> 上手见 [§0](00_guide.md)；分层、数据流与实现状态见 [§2 架构](02_architecture.md)。

| 本文 §1 | 相关文档 |
|---------|----------|
| 定位与能力 | [§0 指南](00_guide.md) · [§2 架构](02_architecture.md) · [§6 综述](06_survey.md) |

---

## 1.1 定位

| 维度 | 说明 |
|------|------|
| 层级 | 外部接口层（External Bridge） |
| 输入 | 外部导航/探索指令、状态订阅请求 |
| 输出 | `NavigationCommandResponse`、`ExplorationCommandResponse`、机器人状态/事件流 |
| 下游 | Navigator、Exploration、Localization、Visualization |
| 对标 | ROS 2 `rosbridge_suite`、Nav2 gRPC 封装、Apollo Cyber RT Bridge |

Bridge **不参与**路径规划或运动控制，只将外部命令译为内部服务调用，并将内部状态以 Protobuf / MQTT 载荷对外发布。

---

## 1.2 核心能力

- **gRPC**：`async_grpc::Server` + `AutonomyService`（导航、探索、状态流）
- **MQTT**（规划中）：轻量 Pub/Sub，适合 IoT / 弱网
- **配置驱动**：Lua → `BridgeOptions`，与全栈一致
- **插件化传输**：`GrpcBridgeServer` / `MqttBridge` 由 `BridgeServer` 调度
- **指令 FSM**：导航与探索命令的有限状态机，见 [RPC · AutonomyService](rpcs/01_autonomy_service.md) 与 [§6 综述](06_survey.md)

---

## 1.3 相关模块

| 模块 | 关系 |
|------|------|
| [Navigator](../16_Navigator/index.rst) | 接收 `NAV_CMD_START` 等指令与目标位姿 |
| [commsgs](../14_Commsgs/index.rst) | `geometry_msgs`、`vehicle_msgs` |
| [Communication](../03_Communication/index.rst) | 机载栈内 `autolink` 通信 |
| `autonomy/common/async_grpc` | 异步 gRPC 框架（源自 Cartographer） |
| `config/bridge/bridge.lua` | 运行时配置 |

源码目录与依赖见 [§2.7](02_architecture.md#27-扩展与依赖)。

---

**导航**：[← §0 指南](00_guide.md) · [§2 架构 →](02_architecture.md)
