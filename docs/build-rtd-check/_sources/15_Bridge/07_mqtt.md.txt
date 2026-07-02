# 7. MQTT 桥接插件

MQTT 插件位于 `autonomy/bridge/plugins/mqtt/`，设计目标是为**低带宽、高延迟、多订阅者**场景提供轻量级桥接，与 gRPC 插件并行运行。

> 当前 **实现为空壳**（`mqtt_bridge.hpp/cpp` 仅有命名空间占位）。本文描述**规划设计与预期行为**，便于后续开发对齐。

## 7.1 设计定位

| 维度 | gRPC 插件 | MQTT 插件 |
|------|-----------|-----------|
| 传输 | HTTP/2 + Protobuf | TCP + 二进制/JSON |
| 模式 | RPC + Stream | Pub/Sub |
| 延迟 | 低（局域网 ms 级） | 中（取决于 Broker） |
| 防火墙穿透 | 需开放端口 | 出站连接 Broker 即可 |
| 多客户端 | 每连接独立 Stream | Topic 天然广播 |
| 适用场景 | 移动端 SDK、云平台 API | IoT 遥测、弱网遥控 |

## 7.2 规划架构

```
外部 MQTT 客户端                    Autonomy
      │                                │
      │  PUBLISH autonomy/cmd/nav      │
      ├──────────────────────────────► │ MqttBridge::OnMessage()
      │                                │   └─► FSM → Navigator
      │                                │
      │  SUBSCRIBE autonomy/state      │
      │ ◄──────────────────────────────┤ MqttBridge::PublishState()
      │                                │
      ▼                                ▼
   MQTT Broker (mosquitto / EMQX)
```

## 7.3 Topic 命名规范（草案）

| Topic | 方向 | QoS | 载荷格式 | 说明 |
|-------|------|-----|----------|------|
| `autonomy/{robot_id}/cmd/navigation` | 入站 | 1 | JSON / Protobuf | 导航指令 |
| `autonomy/{robot_id}/cmd/exploration` | 入站 | 1 | JSON / Protobuf | 探索指令 |
| `autonomy/{robot_id}/state/robot` | 出站 | 0 | Protobuf | 机器人状态（高频） |
| `autonomy/{robot_id}/event` | 出站 | 1 | JSON | 事件告警（低频可靠） |
| `autonomy/{robot_id}/ack/{cmd_id}` | 出站 | 1 | JSON | 指令确认 |

`{robot_id}` 来自配置或环境变量，支持多机群部署。

## 7.4 消息载荷

### 7.4.1 导航指令（JSON 草案）

```json
{
  "cmd_id": "uuid-v4",
  "timestamp": { "sec": 1719900000, "nanosec": 0 },
  "command": "NAV_CMD_START",
  "poses": [
    { "x": 1.0, "y": 2.0, "theta": 0.0 }
  ]
}
```

与 gRPC `NavigationCommandRequest` 字段一一对应，共享同一 FSM 处理函数。

### 7.4.2 状态上报

高频状态（位姿、电量、速度）使用 QoS 0 降低 Broker 负载：

$$
\lambda_{\text{state}} \approx 10\text{-}50\,\mathrm{Hz}
$$

事件类消息（导航完成、故障）使用 QoS 1 保证至少一次投递。

## 7.5 配置

```lua
-- config/bridge/bridge.lua
mqtt = {
    host = "127.0.0.1",
    port = 1883,          -- 标准 MQTT 端口（当前 lua 为 12345，待统一）
    client_id = "autonomy_bridge",
    username = "",
    password = "",
    keep_alive = 60,
    qos_default = 1,
}
```

对应 `proto::MqttOptions`（当前仅 `host` + `port`，扩展字段待补充）。

## 7.6 与 BridgeServer 集成

```cpp
void BridgeServer::Start() {
    if (options_.use_grpc()) {
        grpc_bridge_->Start();
    }
    if (options_.use_mqtt()) {
        mqtt_bridge_->Start();  // 待实现
    }
}
```

gRPC 与 MQTT 可**同时启用**，共享同一套 FSM 与 Navigator 调用逻辑，避免重复实现。

## 7.7 实现路线图

| 阶段 | 内容 |
|------|------|
| P0 | `MqttBridge` 连接 Broker，订阅 cmd topic |
| P1 | JSON 解析 → 复用 Navigation FSM |
| P2 | 状态 topic 定时发布 `RobotState` |
| P3 | Protobuf 载荷、TLS、用户名密码认证 |
| P4 | 与 gRPC 共享 `BridgeContext`，统一指标 |

## 7.8 安全考虑

| 风险 | 缓解 |
|------|------|
| 未授权发布 | Broker ACL + 用户名密码 / 证书 |
| 指令重放 | `cmd_id` 去重 + `timestamp` 窗口校验 |
| 中间人攻击 | MQTTS（TLS over MQTT） |
| Topic 枚举 | 随机 `robot_id` 后缀或 VPN 隔离 |

## 7.9 延迟与带宽估算

设状态消息大小 $S_{\text{state}} \approx 200$ 字节，频率 $f = 20\,\mathrm{Hz}$：

$$
B_{\text{state}} = S_{\text{state}} \times f \approx 4\,\mathrm{KB/s}
$$

MQTT 协议开销约 2–5 字节/帧 + TCP/IP 头，总带宽：

$$
B_{\text{total}} \approx B_{\text{state}} \times 1.1 + B_{\text{cmd}}
$$

弱网（$B_{\text{avail}} < 10\,\mathrm{KB/s}$）时建议降低状态频率或仅发布差分（delta pose）。
