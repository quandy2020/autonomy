(mqtt-bridge-integration)=
# Bridge 集成

> [§5 MQTT 总览](../05_mqtt.md) · 本专题 **mqtt/05**（H2 **5.x**）。

本章描述 `autonomy/bridge/plugins/mqtt/` 如何接入 [mqtt/03 libmosquitto](03_libmosquitto_api.md) 与 [mqtt/02 Broker](02_mosquitto_broker.md)，并与 [§4 gRPC](../04_grpc.md) 并行运行。

## 5.1 目标架构

```
BridgeServer
    ├── GrpcBridgeServer     (use_grpc)  ← 已实现骨架
    └── MqttBridge           (use_mqtt)  ← 规划中
            ├── libmosquitto client
            ├── SUB  autonomy/{id}/cmd/#
            ├── PUB  state / event / ack
            └── CommandFsm → NavigatorStub（与 gRPC 共享）
```

## 5.2 当前源码状态

| 文件 | 状态 |
|------|------|
| `plugins/mqtt/mqtt_bridge.hpp` | 空命名空间占位 |
| `plugins/mqtt/mqtt_bridge.cpp` | 空文件 |
| `bridge_server.hpp` | 无 `mqtt_bridge_` 成员 |
| `bridge_server.cpp` | `use_mqtt` 仅 `LOG`，未构造插件 |
| `proto::MqttOptions` | 仅 `host`、`port` |

## 5.3 BridgeServer 集成（规划）

```cpp
// bridge_server.hpp（规划）
class BridgeServer {
    // ...
    plugins::grpc::GrpcBridgeServer::UniquePtr grpc_bridge_{nullptr};
    plugins::mqtt::MqttBridge::UniquePtr mqtt_bridge_{nullptr};  // 新增
};

// bridge_server.cpp（规划）
BridgeServer::BridgeServer(const proto::BridgeOptions& options)
    : options_{options} {
    if (options_.use_grpc()) {
        grpc_bridge_ = std::make_unique<plugins::grpc::GrpcBridgeServer>(
            options_.grpc());
    }
    if (options_.use_mqtt()) {
        mqtt_bridge_ = std::make_unique<plugins::mqtt::MqttBridge>(
            options_.mqtt());
    }
}

void BridgeServer::Start() {
    if (grpc_bridge_) grpc_bridge_->Start();
    if (mqtt_bridge_) mqtt_bridge_->Start();
}
```

## 5.4 MqttBridge 生命周期（规划）

```
构造 → mosquitto_new → connect(Broker)
     → subscribe(cmd/#)
     → mosquitto_loop_start
     → [运行：OnMessage → FSM → Navigator]
     → loop_stop → disconnect → destroy
```

| 阶段 | API |
|------|-----|
| 启动 | `mosquitto_connect` + `mosquitto_subscribe` + `mosquitto_loop_start` |
| 收指令 | `on_message` → 解析 JSON → `CommandFsm::HandleNavigation` |
| 发状态 | 定时器 / Navigator 回调 → `mosquitto_publish` QoS 0 |
| 发 ack | FSM 进度 → `mosquitto_publish` `ack/{cmd_id}` QoS 1 |
| 停止 | `mosquitto_loop_stop` + `mosquitto_disconnect` |

## 5.5 配置映射

| `bridge.lua` / `MqttOptions` | libmosquitto | 当前 |
|------------------------------|--------------|------|
| `mqtt.host` | `mosquitto_connect(host, …)` | ✅ proto 已有 |
| `mqtt.port` | `mosquitto_connect(…, port, …)` | ✅ proto 已有 |
| `client_id` | `mosquitto_new(id, …)` | ⏳ 待扩展 proto |
| `username` / `password` | `mosquitto_username_pw_set` | ⏳ 待扩展 |
| `keep_alive` | `connect(…, keepalive)` | ⏳ 待扩展 |
| TLS | `mosquitto_tls_set` | ⏳ 待扩展 |

```lua
-- config/bridge/bridge.lua（目标形态）
mqtt = {
    host = "127.0.0.1",
    port = 1883,                    -- Mosquitto 默认；勿与 gRPC 5005 混淆
    client_id = "autonomy_bridge",
    robot_id = "robot_01",
    username = "bridge_robot_01",
    password = "",                  -- 生产用 secret 注入
    keep_alive = 60,
}
```

## 5.6 与 gRPC 共存

| 场景 | 行为 |
|------|------|
| 仅 gRPC | `use_grpc=true, use_mqtt=false`（当前默认） |
| 仅 MQTT | 弱网 IoT、仅 Broker 出站 |
| 双栈 | 移动端 gRPC + 云平台 MQTT 同时接入；**FSM 需串行化**或按来源分区，避免双通道并发 START |

建议：共享 `NavigatorStub` 与 `BridgeContext`，在 FSM 层互斥或拒绝并发活动会话。

## 5.7 实现路线图

| 阶段 | 内容 | 验证 |
|------|------|------|
| P0 | 引入 `libmosquitto` 依赖；`MqttBridge::Start` 连接 + SUB | `mosquitto_sub` 见连接日志 |
| P1 | JSON 解析 `cmd/navigation` → 打日志 | `mosquitto_pub` 发 STOP |
| P2 | 接入 CommandFsm + `NavigatorStub` | 与 gRPC Handler 行为一致 |
| P3 | PUB `state/robot` @ 20Hz | App `mosquitto_sub` 收状态 |
| P4 | TLS、完整 `MqttOptions`、指标 | MQTTS + ACL |

## 5.8 开发 checklist

- [ ] Bazel/CMake 添加 `libmosquitto`
- [ ] 扩展 `MqttOptions` proto（`client_id`、`robot_id`、auth、TLS）
- [ ] 实现 `MqttBridge`：`loop_start`、回调、优雅退出
- [ ] Topic 与 [mqtt/04](04_topic_protocol.md) 一致；Broker ACL 与 [mqtt/02 §2.4](02_mosquitto_broker.md#24-topic-acl-示例) 一致
- [ ] 与 gRPC 共享 FSM，避免重复业务逻辑
- [ ] `BridgeServer` 按 `use_mqtt` 构造 / 启动
- [ ] 文档：更新 [grpc/07 Handler 对照表](../grpc/07_handlers.md#71-handler-对照表)

---

**导航**：[← 04 Topic 协议](04_topic_protocol.md) · [06 上游参考 →](06_upstream_reference.md)
