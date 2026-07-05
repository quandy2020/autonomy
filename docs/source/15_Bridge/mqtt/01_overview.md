(mqtt-overview)=
# Mosquitto

> [§5 MQTT 总览](../05_mqtt.md) · 本专题 **mqtt/01**（H2 **1.x**）。文档地图见 [00_guide §0.1](../00_guide.md#01-文档地图)。

## 1.1 MQTT 协议

MQTT（Message Queuing Telemetry Transport）是面向 IoT 的**轻量 Pub/Sub** 协议，由 Broker 中转消息，客户端无需彼此直连。

| 维度 | 说明 |
|------|------|
| 模式 | 发布 / 订阅（Publish-Subscribe） |
| 传输 | 通常 TCP；TLS 加密为 MQTTS |
| 载荷 | 二进制或文本（Bridge 规划 JSON / Protobuf） |
| QoS | 0（至多一次）、1（至少一次）、2（恰好一次） |
| 标准 | [MQTT 3.1.1](https://docs.oasis-open.org/mqtt/mqtt/v3.1.1/mqtt-v3.1.1.html) · [MQTT 5.0](https://docs.oasis-open.org/mqtt/mqtt/v5.0/mqtt-v5.0.html) |

与 gRPC 对比见 [§5.1](../05_mqtt.md#51-组件)。

## 1.2 Eclipse Mosquitto

[Eclipse Mosquitto](https://mosquitto.org/) 是 Eclipse 基金会下的**开源 MQTT Broker**（EPL/EDL），实现 MQTT 5.0 / 3.1.1 / 3.1。Autonomy 选用其 **libmosquitto** 客户端库与文档体系作为 Bridge MQTT 插件的参考实现；Broker 本身可替换为 EMQX、HiveMQ、AWS IoT Core 等协议兼容产品。

| 组件 | 路径 / 工具 | Bridge 用途 |
|------|-------------|-------------|
| **Broker** | `mosquitto` 守护进程 | 场外 / 边缘消息中枢；Bridge **不内嵌** Broker |
| **C 客户端库** | `libmosquitto` | `MqttBridge` 首选依赖（订阅 cmd、发布 state） |
| **CLI 工具** | `mosquitto_pub` / `mosquitto_sub` | 联调、CI 冒烟 |
| **配置** | `mosquitto.conf` | Listener、认证、TLS、持久化 |

源码：[github.com/eclipse-mosquitto/mosquitto](https://github.com/eclipse-mosquitto/mosquitto)

## 1.3 Bridge 角色

```
┌─────────────────┐     PUBLISH cmd      ┌──────────────────┐     SUBSCRIBE cmd
│ 移动端 / 调度    │ ──────────────────► │  Mosquitto Broker │ ◄───────────────── MqttBridge
│ (MQTT Client)   │ ◄────────────────── │  (独立进程)        │ ─────────────────► PUBLISH state
└─────────────────┘     SUBSCRIBE state └──────────────────┘
                                              ▲
                                              │ TCP :1883 / :8883
                                              │
                                    autonomy/bridge/plugins/mqtt
                                    (libmosquitto 客户端，规划中)
```

| 角色 | 进程 | 说明 |
|------|------|------|
| MQTT Broker | `mosquitto` | 消息路由、QoS、会话、ACL |
| 场外客户端 | App / 云平台 | 发布导航指令、订阅机器人状态 |
| `MqttBridge` | Autonomy 机载 | 订阅指令 Topic → FSM → Navigator；发布状态 Topic |

MQTT 层只负责传输；业务语义与 [§3 RPC](../rpcs/02_service_overview.md) 共享同一 FSM（见 [mqtt/05 Bridge 集成](05_bridge_integration.md)）。实现状态见 [grpc/07 §7.1](../grpc/07_handlers.md#71-handler-对照表)。

---

**导航**：[← §5 总览](../05_mqtt.md) · [02 Broker 部署 →](02_mosquitto_broker.md)
