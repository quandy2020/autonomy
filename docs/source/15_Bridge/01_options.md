---
orphan: true
---
(bridge-options)=
# 1. 参数配置

> **§1** · `bridge_options.proto` · 机载部署配置（**非** AutonomyService RPC）。集成方远程调 API 通常无需阅读本章；入口见 [§0.3 配置入口](00_guide.md#03-配置入口)。

`bridge_options.proto`：Lua → `proto::BridgeOptions` → `BridgeServer`。

## 1.1 消息结构

```protobuf
message BridgeOptions {
  bool use_grpc = 1;
  bool use_mqtt = 2;
  GrpcOptions grpc = 3;
  MqttOptions mqtt = 4;
}
```

## 1.2 GrpcOptions

| 字段 | 说明 | `bridge.lua` 默认 |
|------|------|-------------------|
| `host` / `port` | 监听地址 | `127.0.0.1` / `5005` |
| `num_grpc_threads` / `num_event_threads` | 工作 / CQ 线程 | `5` / `5` |
| `enable_ssl_encryption` | TLS | — |
| `uplink_server_address` 等 | 上行预留 | — |

> **已知限制**：`GrpcBridgeServer` 尚未将字段传入 `async_grpc::Server::Builder`，当前硬编码 `127.0.0.1`、4 线程。见 [§4.2](04_grpc.md#42-服务器构建)。

```lua
AUTONOMY_BRIDGE = {
    use_grpc = true,
    grpc = { host = "127.0.0.1", port = 5005 },
}
```

## 1.3 MqttOptions

| 字段 | 默认 |
|------|------|
| `host` / `port` | `127.0.0.1` / `12345` |

MQTT 插件未实现；Topic 见 [§5 MQTT](05_mqtt.md) · [mqtt/04](mqtt/04_topic_protocol.md)。

---

**导航**：[← §0.3 配置入口](00_guide.md#03-配置入口) · [§2 架构设计 →](02_architecture.md)
