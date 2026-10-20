(bridge-survey)=
# 5. 协议综述

选型与能力边界。实现见 [§2](02_architecture.md) · 契约见 [§3](03_rpc_protocol.md)。

### 5.1 能力边界

| 能力 | 状态 |
|------|------|
| gRPC + `automsgs.rpcs.*` | — |
| async_grpc 服务端 | （Handler 按域陆续接线） |
| BotStates / BotEvents Stream | — |
| MQTT | 已移除 |
| TLS / 认证 / uplink | Proto 预留 |

明细：[grpc/07](grpc/07_handlers.md)。

### 5.2 交互模式

| 模式 | 映射 |
|------|------|
| Server Streaming | Navigate / Explore / Follow / Charge 等 |
| Bidi | Teleop |
| Unary | Get* / Estop / CancelAll |

### 5.3 业界对照

| 方案 | 传输 | 与 Autonomy |
|------|------|-------------|
| rosbridge | WS + JSON | 弱类型 |
| foxglove_bridge | WS + Protobuf | 可视化生态 |
| Apollo Gateway | gRPC / HTTP | async_grpc 同源 |
| Autoware AD API | REST / WS | ROS 2 |
| Nav2 commander | 进程内 | 无跨网契约 |

Autonomy：**gRPC + Protobuf** 为主。

### 5.4 选型

| 场景 | 推荐 |
|------|------|
| 局域网 App | gRPC + TLS |
| 云端调度 | gRPC Stream |
| 本机调试 | `127.0.0.1:5005` |

### 5.5 工程约束（摘要）

| 主题 | 要点 |
|------|------|
| 延迟 | 遥控 < 200 ms；导航启动 < 2 s |
| 背压 | Stream 降采样或断慢连接 |
| 重连 | 指数退避，上限约 30 s |
| 安全 | 默认 localhost；生产 mTLS |
