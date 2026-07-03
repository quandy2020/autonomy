(bridge-bridge-options)=
# 2. BridgeOptions

`bridge_options.proto` 定义 Bridge **运行时配置**的 Protobuf 形态，由 `config/bridge/bridge.lua` 经 `LoadOptions()` 解析。此文件**不是** gRPC 服务，但与 `AutonomyService` 同属 `autonomy.bridge.proto` 包。

| 本文 §2 | 相关文档 |
|---------|----------|
| 配置字段 | [§0 Bridge 指南 · 配置](../00_guide.md#05-配置与-api) · [§0 RPC 指南](00_guide.md) |

---

## 2.1 消息结构

```protobuf
message BridgeOptions {
  bool use_grpc = 1;
  bool use_mqtt = 2;
  GrpcOptions grpc = 3;
  MqttOptions mqtt = 4;
}
```

`BridgeServer` 根据 `use_grpc` / `use_mqtt` 决定是否构造 `GrpcBridgeServer` / `MqttBridge`。

---

## 2.2 GrpcOptions

| 字段 | 类型 | 说明 | `bridge.lua` 默认 |
|------|------|------|-------------------|
| `host` | `string` | gRPC 监听地址 | `127.0.0.1` |
| `port` | `uint32` | 监听端口 | `5005` |
| `num_grpc_threads` | `uint32` | gRPC 工作线程 | `5` |
| `num_event_threads` | `uint32` | Completion Queue 事件线程 | `5` |
| `uplink_server_address` | `string` | 上行云端 gRPC 地址（预留） | — |
| `upload_batch_size` | `int32` | 批量上传大小（预留） | — |
| `enable_ssl_encryption` | `bool` | TLS | — |
| `enable_google_auth` | `bool` | Google 认证（预留） | — |

> **已知限制**：`GrpcBridgeServer` 构造时尚未将上述字段传入 `async_grpc::Server::Builder`，当前硬编码 `127.0.0.1`、4 线程。字段语义以 proto 与 Lua 为准，接线完成后与 [§3.2](../03_grpc.md#32-服务器构建) 一致。

**Lua 示例**

```lua
AUTONOMY_BRIDGE = {
    use_grpc = true,
    use_mqtt = false,
    grpc = {
        host = "127.0.0.1",
        port = 5005,
        num_grpc_threads = 5,
        num_event_threads = 5,
    },
}
```

**C++ 加载**

```cpp
auto options = autonomy::bridge::common::LoadOptions(bridge_dict);
// options 类型: autonomy::bridge::proto::BridgeOptions
```

---

## 2.3 MqttOptions

| 字段 | 类型 | 说明 | 默认 |
|------|------|------|------|
| `host` | `string` | MQTT Broker 主机 | `127.0.0.1` |
| `port` | `uint32` | Broker 端口 | `12345`（Lua 侧约定） |

MQTT 插件尚未实现；Topic 与载荷规划见 [§4 MQTT](../04_mqtt.md)。

---

## 2.4 与 AutonomyService 的关系

| 层级 | 产物 | 消费者 |
|------|------|--------|
| `BridgeOptions` | 进程内配置 | `BridgeServer`、`GrpcBridgeServer` |
| `AutonomyService` | 对外 RPC | 移动端、云平台、第三方 SDK |

客户端连接地址由 `GrpcOptions.host` + `GrpcOptions.port` 决定；RPC 路径与方法名见 [§1 AutonomyService](01_autonomy_service.md)。

---

**导航**：[← §1 AutonomyService](01_autonomy_service.md) · [§0 Bridge 指南](../00_guide.md)
