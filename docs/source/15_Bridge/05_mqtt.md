(bridge-mqtt)=
# 5. MQTT 插件

MQTT 插件（`plugins/mqtt/`）与 [§4 gRPC](04_grpc.md) 并行：机载 `MqttBridge` 为 **libmosquitto 客户端**，经 [Eclipse Mosquitto](https://mosquitto.org/) Broker 与场外 Pub/Sub 通信，语义对齐 [§3 RPC](03_rpc_protocol.md)。**当前为空壳**。

> **编号约定**：本页 **§5.1–§5.6** 为插件总览；`mqtt/01_*`–`06_*` 为 Mosquitto 专题（子页 H2 为 `{文件前缀}.x`，如 `02` → `## 2.1`）。文档地图见 [00_guide §0.1](00_guide.md#01-文档地图)。

## 5.1 组件

| 路径 | 职责 |
|------|------|
| `plugins/mqtt/mqtt_bridge.*` | `MqttBridge` 生命周期、SUB/PUB（规划中） |
| `bridge_server.*` | 按 `use_mqtt` 构造 / 启动插件（待接线） |
| `proto/bridge_options.proto` | `MqttOptions`（`host`、`port`） |

| 维度 | gRPC | MQTT |
|------|------|------|
| Autonomy 角色 | **Server** | **Client**（连 Broker） |
| 模式 | RPC + Stream | Pub/Sub |
| 防火墙 | 开放 Bridge 端口 | 机载出站连 Broker |

## 5.2 架构

```
App ──PUBLISH cmd──► Mosquitto Broker ◄──SUB cmd/#── MqttBridge ──► Navigator
App ◄──SUB state──── Broker ◄──PUB state──── MqttBridge
```

Broker **独立部署**（Bridge 不内嵌）；角色划分见 [mqtt/01 §1.3](mqtt/01_overview.md#13-bridge-角色)。

## 5.3 Topic

Topic 摘要（完整见 [mqtt/04](mqtt/04_topic_protocol.md)）：

| Topic | 对应 gRPC | QoS |
|-------|-----------|-----|
| `cmd/navigation` … `cmd/map` | `Send*Command` | 1 |
| `cmd/emergency_stop` | `EmergencyStop` | 1 |
| `state/robot` | `ReceiveBotStates` | 0 |
| `event` | `ReceiveBotEvents` | 1 |
| `ack/{cmd_id}` | Command Stream 响应 | 1 |

## 5.4 配置与状态

```lua
mqtt = { host = "127.0.0.1", port = 1883 }  -- Broker 地址，非 gRPC 5005
```

| 组件 | 状态 |
|------|------|
| `MqttBridge` / `BridgeServer` 调度 | ❌ 空壳 / 未接线 |
| 专题文档 | ✅ [mqtt/](mqtt/index.rst) |

实现明细：[grpc/07 §7.1](grpc/07_handlers.md#71-handler-对照表) · 路线图：[mqtt/05 §5.7](mqtt/05_bridge_integration.md#57-实现路线图)。

## 5.5 Mosquitto 专题

| 文件 | 内容 |
|------|------|
| [mqtt/01](mqtt/01_overview.md) | MQTT 协议、Mosquitto 组件、Bridge 角色 |
| [mqtt/02](mqtt/02_mosquitto_broker.md) | Broker 安装、配置、ACL、TLS |
| [mqtt/03](mqtt/03_libmosquitto_api.md) | libmosquitto API、`MqttBridge` 封装 |
| [mqtt/04](mqtt/04_topic_protocol.md) | Topic 命名、MQTT↔RPC、JSON 载荷 |
| [mqtt/05](mqtt/05_bridge_integration.md) | `BridgeServer` 集成、双栈共存 |
| [mqtt/06](mqtt/06_upstream_reference.md) | Mosquitto 官方资源 |
| [rpcs/05 §5.3](rpcs/05_stream_api.md#53-vehicle_msgs) | `RobotState` / `RobotEvent` 字段 |

## 5.6 快速联调

```bash
mosquitto_sub -h 127.0.0.1 -p 1883 -t 'autonomy/robot_01/state/robot' -v
mosquitto_pub -h 127.0.0.1 -p 1883 -t 'autonomy/robot_01/cmd/navigation' -q 1 \
  -m '{"header":{"cmd_id":"t1"},"command":"NAV_CMD_STOP"}'
```

Broker 与 ACL：[mqtt/02](mqtt/02_mosquitto_broker.md) · 载荷示例：[mqtt/04 §4.3](mqtt/04_topic_protocol.md#43-json-示例)。

---

**导航**：[← §4 gRPC](04_grpc.md) · [mqtt/ 专题 →](mqtt/index.rst) · [§6 综述 →](06_survey.md)
