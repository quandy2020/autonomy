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
  GrpcOptions grpc = 3;
}
```

## 1.2 GrpcOptions

| 字段 | 说明 | `bridge.lua` 默认 |
|------|------|-------------------|
| `host` / `port` | 监听地址 | `127.0.0.1` / `5005` |
| `num_grpc_threads` / `num_event_threads` | 工作 / CQ 线程 | `5` / `5` |
| `enable_ssl_encryption` | TLS | — |
| `uplink_server_address` 等 | 上行预留 | — |

```lua
AUTONOMY_BRIDGE = {
    use_grpc = true,
    grpc = { host = "127.0.0.1", port = 5005 },
}
```

---

**导航**：[← §0.3 配置入口](00_guide.md#03-配置入口) · [§2 架构设计 →](02_architecture.md)
