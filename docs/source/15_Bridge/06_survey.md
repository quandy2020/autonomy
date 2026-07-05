(bridge-survey)=
# 6. 协议综述

> **本文范围**：Bridge **协议谱系、业界方案、选型依据**与 Autonomy 能力边界。  
> 架构与实现见 [§2](02_architecture.md)；契约见 [§3](03_rpc_protocol.md)；插件见 [§4 gRPC](04_grpc.md) · [§5 MQTT](05_mqtt.md)。

---

## 6.1 综述定位

| 维度 | 本文 | 其他文档 |
|------|------|----------|
| 分层、数据流、Handler 接线 | 不展开 | [§2 架构](02_architecture.md) |
| Proto 字段、Topic 对照 | 摘要 + 链接 | [§3](03_rpc_protocol.md) · [rpcs/](rpcs/index.rst) · [mqtt/04](mqtt/04_topic_protocol.md) |
| async_grpc / Mosquitto 实现 | 摘要 + 链接 | [grpc/](grpc/index.rst) · [mqtt/](mqtt/index.rst) |
| 协议分类、业界对比、选型 | **本文** | — |

**建议阅读顺序**

| 角色 | 路径 |
|------|------|
| 选型 / 集成 | §6.2 → §6.7 决策树 |
| 协议设计 | §6.3 → §6.5 设计原则 → §6.6 工程约束 |
| 背景调研 | §6.4 → §6.8 参考 |

---

## 6.2 Autonomy 能力边界

Bridge 位于**机载栈**（Localization / Planning / Control / Navigator）与**场外系统**（App、云平台、调度）之间，对标 `rosbridge_suite`、`foxglove_bridge` 及 Apollo / Autoware 的对外 API 层。

```
Offboard ── gRPC / MQTT ──► autonomy/bridge ──► Navigator / Exploration
Onboard  ◄── autolink / commsg ────────────────────────────────────────┘
```

| 能力 | 状态 | 说明 |
|------|------|------|
| gRPC + Protobuf 契约 | ✅ | `AutonomyService` **13 RPC** 已定义 |
| async_grpc 服务端骨架 | ✅ | 2 Handler 已注册（Nav / Expl） |
| 其余 Command / Query / System | ❌ | Proto 有，未 RegisterHandler |
| BotStates / BotEvents Stream | ❌ | `vehicle_msgs` 已定义，Handler 未注册 |
| MQTT（libmosquitto 客户端） | ❌ | 空壳 |
| TLS / 认证 / uplink | ⏳ | Proto 预留字段 |

实现明细见 [grpc/07 §7.1](grpc/07_handlers.md#71-handler-对照表)（上表为能力边界摘要）。

---

## 6.3 协议谱系

### 6.3.1 按层次

| 层次 | 代表 | Autonomy |
|------|------|----------|
| 应用 RPC | gRPC, REST, WebSocket | ✅ gRPC |
| 消息中间件 | MQTT, AMQP, Kafka | ⏳ MQTT |
| 机器人中间件 | DDS (ROS 2), Cyber RT | 内部 autolink |
| 传输 | TCP / HTTP/2 / QUIC | gRPC → HTTP/2 → TCP |

### 6.3.2 按交互模式

| 模式 | Bridge 映射 |
|------|-------------|
| Server Streaming | `Send*Command`（Nav/Expl/Follow/Dock/Map） |
| Bidirectional Streaming | `SendTeleopCommand` |
| Empty → Stream | `ReceiveBotStates` / `ReceiveBotEvents` |
| Unary | `Get*` / `EmergencyStop` / `CancelAllTasks` |

### 6.3.3 序列化

| 格式 | Bridge 使用 |
|------|-------------|
| Protobuf | gRPC 主格式 |
| JSON | MQTT 载荷备选 |
| commsg / ROS msg | 机载内部，不经 Bridge 暴露 |

---

## 6.4 业界方案

| 方案 | 传输 | 特点 | 与 Autonomy 差异 |
|------|------|------|------------------|
| `rosbridge_suite` | WebSocket + JSON | 浏览器友好 | 弱类型、性能一般 |
| `foxglove_bridge` | WebSocket + Protobuf | 可视化集成 | 偏 Foxglove 生态 |
| Apollo Cyber RT | gRPC / HTTP Gateway | 多模块 DAG | async_grpc 同源参考 |
| Autoware AD API | REST / WebSocket | 标准化对外 API | ROS 2 绑定 |
| Nav2 `simple_commander` | 进程内 Python | 非网络 Bridge | 无跨网契约 |

Autonomy 选型：**gRPC 强类型 RPC 为主，MQTT Pub/Sub 为辅**；与 Navigator 解耦，独立 `autonomy/bridge` 模块。

---

## 6.5 设计原则

**命令 vs 推送**：外部接口区分有副作用的 **Command**（`NAV_CMD_*`，须 FSM）、无副作用的 **Query**（可重试）、单向 **Event**（Stream 推送）。当前 Proto 以命令 + Stream 为主，近似 CQRS。

**幂等**：弱网重发时建议 `cmd_id` + 服务端滑动窗口去重（建议保留 60 s）。

**多点导航**：`repeated PoseStamped goals` + `NavigationMode`（单点 / 多点巡航）；执行由 Navigator 封装，Bridge 透传。

---

## 6.6 工程约束

| 主题 | 要点 |
|------|------|
| 延迟 | 遥控端到端 < 200 ms；导航启动 < 2 s（含 RTT + FSM + Navigator） |
| 背压 | Stream 推送速率超过客户端消费时降采样或断开慢连接，防 OOM |
| 重连 | gRPC 客户端指数退避（$T_0 \cdot 2^k$，上限约 30 s） |
| 安全 | 默认 `127.0.0.1`；生产 mTLS + 反向代理；`timestamp` + nonce 防重放 |
| 可观测 | `bridge_rpc_total` / `bridge_rpc_latency_ms` / `bridge_active_streams` / `bridge_fsm_reject_total` |

配置与安全实践见 [mqtt/02](mqtt/02_mosquitto_broker.md) · [§4 gRPC](04_grpc.md)。

---

## 6.7 选型决策树

```
需要浏览器直连？
├── 是 → WebSocket（rosbridge）或 gRPC-Web
└── 否 → 强类型 RPC？
         ├── 是 → gRPC + Protobuf ✅（Autonomy 默认）
         └── 否 → IoT 广播 / 弱网？
                  ├── 是 → MQTT ✅（规划）
                  └── 否 → REST / 自定义 TCP
```

| 场景 | 推荐 | 备注 |
|------|------|------|
| 局域网移动端 | gRPC + TLS | `host:0.0.0.0:5005` |
| 云端调度 | gRPC Stream | `uplink_server_address` |
| 多机监控 | MQTT | QoS 0 状态 + QoS 1 事件 |
| 调试原型 | gRPC localhost | 当前默认硬编码待修复 |
| 弱网 | MQTT | 低频 + 差分状态 |

---

## 6.8 路线与参考

| 阶段 | 目标 |
|------|------|
| v0.1 | gRPC 骨架 + Proto | ✅ |
| v0.2 | Handler 接线 Navigator | ⏳ |
| v0.3 | BotStates Stream | 规划 |
| v0.4 | MQTT 插件 | 规划 |
| v0.5 | TLS / 认证 / uplink | 规划 |
| v1.0 | 接入 `system::Autonomy` | 规划 |

| 参考 | 与 Bridge 关系 |
|------|----------------|
| [rosbridge (IROS 2017)](https://msl.stanford.edu/papers/crick_iros2017.pdf) | WebSocket 互操作架构 |
| [gRPC on HTTP/2](https://grpc.io/blog/grpc-on-http2/) | 线程池与多路复用 |
| [Cartographer Cloud](https://google-cartographer.readthedocs.io/en/latest/cloud_overview.html) | async_grpc / uplink |
| [Apollo Cyber RT](https://apollo.baidu.com/docs/apollo/latest/md_cyber_2README.html) | 高性能中间件 + Gateway |

---

**导航**：[← §5 MQTT](05_mqtt.md) · [§0 指南](00_guide.md)
