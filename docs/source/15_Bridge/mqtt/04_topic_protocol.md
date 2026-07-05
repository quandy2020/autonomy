(mqtt-topic-protocol)=
# Topic 协议

> [§5 MQTT 总览](../05_mqtt.md) · 本专题 **mqtt/04**（H2 **4.x**）。

MQTT 层 **复用 §3 gRPC 消息语义**（JSON / Protobuf 载荷与 proto 字段一致）。Broker 推荐 [Eclipse Mosquitto](https://mosquitto.org/)。

## 4.1 Topic 命名

前缀 `autonomy/{robot_id}/`：

| Topic | 方向 | QoS | 对应 gRPC |
|-------|------|-----|-----------|
| `cmd/navigation` | SUB | 1 | `SendNavigationCommand` |
| `cmd/exploration` | SUB | 1 | `SendExplorationCommand` |
| `cmd/follow` | SUB | 1 | `SendFollowCommand` |
| `cmd/teleop` | SUB | 1 | `SendTeleopCommand`（高频） |
| `cmd/dock` | SUB | 1 | `SendDockCommand` |
| `cmd/map` | SUB | 1 | `SendMapCommand` |
| `cmd/emergency_stop` | SUB | 1 | `EmergencyStop` |
| `query/snapshot` | SUB | 1 | `GetRobotSnapshot`（请求-响应） |
| `state/robot` | PUB | 0 | `ReceiveBotStates` |
| `event` | PUB | 1 | `ReceiveBotEvents` |
| `ack/{cmd_id}` | PUB | 1 | Command Stream 的 `*CommandResponse` |

Wildcard：`autonomy/{robot_id}/cmd/#`。

## 4.2 载荷约定

- 所有 Command 载荷含 **`header`**（`timestamp`、`cmd_id`、`client_id`、`nonce`），对齐 `RequestHeader`
- Stream 响应 PUB 至 `ack/{cmd_id}`，含 **`ack`**（`success`、`message`、`final`、`error_code`）
- `final: true` 对标 gRPC Stream 结束

Proto 字段详见 [rpcs/02 §2.1](../rpcs/02_service_overview.md#21-方法列表)（[07 导航](../rpcs/07_navigation_command.md) 等）。

## 4.3 JSON 示例

**导航 START**

```json
{
  "header": {
    "cmd_id": "550e8400-e29b-41d4-a716-446655440000",
    "timestamp": { "sec": 1719900000, "nanosec": 0 },
    "client_id": "mobile_app"
  },
  "command": "NAV_CMD_START",
  "mode": "NAV_MODE_SINGLE_POSE",
  "goals": [{
    "header": { "frame_id": "map" },
    "pose": { "position": { "x": 1.0, "y": 2.0 }, "orientation": { "w": 1.0 } }
  }],
  "plugins": { "planner_id": "navfn_planner", "controller_id": "graceful_controller" }
}
```

**ack 进度**

```json
{
  "ack": {
    "cmd_id": "550e8400-e29b-41d4-a716-446655440000",
    "success": true,
    "message": "navigating",
    "final": false,
    "task_type": "TASK_TYPE_NAVIGATION",
    "task_status": "TASK_STATUS_RUNNING"
  },
  "status": "NAV_STATUS_NAVIGATING",
  "distance_remaining": 12.5
}
```

## 4.4 QoS 与安全

| 类型 | QoS | 说明 |
|------|-----|------|
| `state/robot` | 0 | 10–50 Hz，允许丢包 |
| `cmd/*` / `event` / `ack/*` | 1 | 至少一次 + `cmd_id` 去重 |
| `cmd/teleop` | 0 或 1 | 高频；弱网可降频 |

防重放：`header.timestamp` + `nonce`；认证见 [mqtt/02 §2.4](02_mosquitto_broker.md#24-topic-acl-示例)。

---

**导航**：[← 03 libmosquitto](03_libmosquitto_api.md) · [05 Bridge 集成 →](05_bridge_integration.md)
