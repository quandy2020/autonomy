# 8. 机器人通信桥接综述（Survey）

本文从**协议体系、架构模式、工程实践、选型依据**四个维度，系统综述移动机器人外部通信桥接领域，并明确 Autonomy `bridge` 模块在其中的定位、能力边界与发展路线。

> 公式与 FSM 推导见 [Bridge 指南 · 数学原理](03_math.md)；实现细节见 [架构设计](05_architecture.md) 与各插件子文档。

---

## 8.1 概述：Bridge 在导航栈中的位置

移动机器人系统通常分为**机载栈**（onboard）与**场外系统**（offboard）。Bridge 模块位于二者边界：

| 层级 | 组件 | 通信范围 |
|------|------|----------|
| 机载 | Localization / Planning / Control | 进程内 / 本地 DDS |
| **边界** | **`bridge`** | **局域网 / 互联网** |
| 场外 | 移动端 App、云平台、调度中心 | 跨网络 |

```
┌─────────────────────────────────────────────────────────┐
│                    Offboard（场外）                       │
│   移动端 App ── gRPC ──┐    云平台 ── MQTT ──┐          │
└────────────────────────┼────────────────────┼───────────┘
                         │                    │
                    ┌────▼────────────────────▼────┐
                    │      autonomy/bridge         │
                    │  GrpcBridgeServer / MqttBridge│
                    └────┬────────────────────┬────┘
                         │                    │
┌────────────────────────┼────────────────────┼───────────┐
│                    Onboard（机载）          │           │
│              Navigator ── Planning ── Control           │
└─────────────────────────────────────────────────────────┘
```

Autonomy `bridge` 对标 ROS 生态中的 `rosbridge_suite`、`foxglove_bridge`，以及 Apollo / Autoware 中的 Cyber RT Bridge、gRPC Gateway。

---

## 8.2 协议体系分类

### 8.2.1 按抽象层次

| 层次 | 代表协议 | 特点 | Autonomy 采用 |
|------|----------|------|---------------|
| 应用 RPC | gRPC, REST, WebSocket | 强类型、双向流 | ✅ gRPC |
| 消息中间件 | MQTT, AMQP, Kafka | 解耦、广播 | ⏳ MQTT（规划） |
| 机器人中间件 | DDS (ROS 2), Cyber RT | 实时、QoS | 内部 autolink |
| 传输层 | TCP, UDP, QUIC | 可靠/不可靠 | gRPC→HTTP/2→TCP |

### 8.2.2 按交互模式

| 模式 | 语义 | 典型用途 | Bridge RPC 映射 |
|------|------|----------|-----------------|
| Unary | 一问一答 | 配置查询 | — |
| Server Streaming | 一问多答 | 状态推送 | `ReceiveBotStates` |
| Client Streaming | 多问一答 | 批量上传 | — |
| Bidirectional Streaming | 双向流 | 遥控遥测 | 可扩展 |

### 8.2.3 序列化格式对比

| 格式 | 大小 | 解析速度 | 模式演进 | Bridge 使用 |
|------|------|----------|----------|-------------|
| Protobuf | 小 | 快 | 良好（proto3） | ✅ 主格式 |
| JSON | 大 | 慢 | 灵活 | MQTT 备选 |
| MessagePack | 中 | 中 | 无 schema | 未采用 |
| ROS msg | 中 | 快 | 需 .msg 编译 | 内部 commsg |

Protobuf 编码效率近似 $S_{\text{JSON}} / S_{\text{Proto}} \approx 1.5 \sim 3$（典型机器人消息）。

---

## 8.3 主流桥接方案对比

### 8.3.1 ROS 生态

| 项目 | 传输 | 特点 | 局限 |
|------|------|------|------|
| `rosbridge_suite` | WebSocket + JSON | 浏览器友好 | 性能一般，无强类型 |
| `foxglove_bridge` | WebSocket + Protobuf | 可视化集成好 | 偏 Foxglove 生态 |
| `grpc_bridge`（社区） | gRPC | 高性能 | 非官方，需自维护 proto |

### 8.3.2 自动驾驶框架

| 框架 | Bridge 方案 | 说明 |
|------|-------------|------|
| Apollo | Cyber RT + HTTP/gRPC Gateway | 多模块 DAG，Bridge 独立进程 |
| Autoware | ROS 2 + API (IV / AD API) | 标准化 REST/WebSocket |
| Nav2 | `nav2_simple_commander` (Python) | 进程内，非网络 Bridge |

### 8.3.3 Autonomy 定位

| 维度 | Autonomy Bridge | rosbridge | Nav2 API |
|------|-----------------|-----------|----------|
| 传输 | gRPC + MQTT | WebSocket | 进程内 Python |
| 类型安全 | Protobuf schema | JSON 弱类型 | Python 类型 |
| 流式状态 | Server Stream | 订阅模拟 | 回调 |
| 与栈耦合 | 独立模块 | ROS 话题镜像 | Navigator 直连 |

---

## 8.4 指令语义设计

### 8.4.1 命令 vs 查询

机器人外部接口应区分：

| 类型 | 特性 | 示例 |
|------|------|------|
| **命令（Command）** | 有副作用、需 FSM | `NAV_CMD_START` |
| **查询（Query）** | 无副作用、可重试 | `GetRobotState` |
| **事件（Event）** | 单向通知 | `NAVIGATION_COMPLETED` |

Autonomy 当前 Proto 以命令为主，状态/事件通过 Stream 推送，符合 **CQRS** 简化变体。

### 8.4.2 幂等性

网络不可靠环境下，客户端可能重发指令。建议对命令引入 `cmd_id`：

$$
\mathrm{Handle}(cmd) = \begin{cases}
\mathrm{noop} & \mathrm{if}\; \mathit{id} \in \mathcal{H} \\
\mathrm{execute} \land \mathcal{H} \leftarrow \mathit{id} & \mathrm{otherwise}
\end{cases}
$$

其中 $\mathit{id}$ 为 `cmd_id`，$\mathcal{H}$ 为已处理命令集合（滑动窗口，建议保留 60 s）。

### 8.4.3 多点导航

`PoseArray` 支持航点序列 $\{q_0, q_1, \ldots, q_N\}$，执行策略：

| 策略 | 说明 | 适用 |
|------|------|------|
| 顺序巡航 | $q_i \to q_{i+1}$ 依次到达 | 巡检 |
| 仅终点 | 忽略中间点 | 简单导航 |
| 行为树封装 | BT Waypoint Follower | 复杂逻辑 |

---

## 8.5 性能与可靠性

### 8.5.1 延迟预算

典型移动操作场景端到端延迟预算：

| 环节 | 预算 |
|------|------|
| 网络 RTT | 10–100 ms |
| Bridge 解析 + FSM | 1–5 ms |
| Navigator 响应 | 10–50 ms |
| **总计** | **< 200 ms**（遥控）；**< 2 s**（导航启动） |

### 8.5.2 背压（Backpressure）

Server Stream 推送速率 $r_{\text{pub}}$ 超过客户端消费速率 $r_{\text{sub}}$ 时：

$$
Q(t) = \int_0^t \bigl(r_{\text{pub}}(\tau) - r_{\text{sub}}(\tau)\bigr) \, d\tau
$$

当 $Q(t) > Q_{\max}$ 时应降采样或断开慢客户端，避免 OOM。

### 8.5.3 断线重连

gRPC 通道状态机：

$$
\mathrm{IDLE} \to \mathrm{CONNECTING} \to \mathrm{READY} \to \mathrm{TRANSIENT}_{\mathrm{FAILURE}} \to \mathrm{RECONNECT}
$$

客户端应实现指数退避重连：

$$
T_{\mathrm{retry},k} = T_0 \cdot 2^k, \quad T_{\max} = 30\,\mathrm{s}
$$

---

## 8.6 安全体系

### 8.6.1 威胁模型

| 威胁 | 影响 | 缓解 |
|------|------|------|
| 未授权指令 | 机器人异常运动 | mTLS + Token |
| 中间人窃听 | 位姿泄露 | TLS 1.3 |
| 重放攻击 | 重复执行旧指令 | timestamp + nonce |
| DDoS | 服务不可用 | 速率限制、连接数上限 |

### 8.6.2 零信任原则

- Bridge 默认绑定 `127.0.0.1`，不对外暴露
- 生产环境通过反向代理（Envoy / nginx）终止 TLS
- `enable_google_auth` 对接 GCP IAM（预留字段）

---

## 8.7 可观测性

| 指标 | 类型 | 说明 |
|------|------|------|
| `bridge_rpc_total` | Counter | RPC 调用总数（按方法分） |
| `bridge_rpc_latency_ms` | Histogram | 端到端延迟 |
| `bridge_active_streams` | Gauge | 当前活跃 Stream 数 |
| `bridge_fsm_reject_total` | Counter | FSM 拒绝次数 |
| `bridge_mqtt_publish_bytes` | Counter | MQTT 出站字节 |

---

## 8.8 选型决策树

```
需要浏览器直接访问？
├── 是 → WebSocket Bridge（rosbridge 风格）或 gRPC-Web
└── 否 → 需要强类型 RPC？
         ├── 是 → gRPC + Protobuf ✅（Autonomy 默认）
         └── 否 → 需要 IoT 广播 / 弱网？
                  ├── 是 → MQTT ✅（Autonomy 规划）
                  └── 否 → REST / 自定义 TCP
```

### 8.8.1 场景推荐

| 场景 | 推荐 | 关键配置 |
|------|------|----------|
| 局域网移动端 | gRPC | `host:0.0.0.0` + TLS |
| 云端调度 | gRPC Stream | `uplink_server_address` |
| 多机器人监控 | MQTT | QoS 0 状态 + QoS 1 事件 |
| 调试原型 | gRPC localhost | `127.0.0.1:5005` |
| 离线弱网 | MQTT | 低频率 + 差分状态 |

---

## 8.9 学术与工业参考

| 文献 / 项目 | 贡献 | 与 Bridge 关系 |
|-------------|------|----------------|
| ROS Bridge Paper (IROS) | WebSocket 机器人互操作 | 架构参考 |
| gRPC Performance Study | HTTP/2 多路复用基准 | 线程池配置依据 |
| MQTT for Robotics (IEEE) | QoS 在机器人中的权衡 | MQTT 插件设计 |
| Apollo Cyber RT | 高性能中间件 + Bridge | async_grpc 同源 |
| Cartographer Cloud | gRPC 地图上传 | uplink 设计参考 |

---

## 8.10 Autonomy Bridge 发展路线

| 版本 | 目标 | 状态 |
|------|------|------|
| v0.1 | gRPC Server 骨架 + Proto 定义 | ✅ 当前 |
| v0.2 | Navigation Handler 接线 Navigator | ⏳ 进行中 |
| v0.3 | BotStates Stream + vehicle_msgs 填充 | 规划 |
| v0.4 | MQTT 插件 + JSON 载荷 | 规划 |
| v0.5 | TLS / 认证 / uplink 云端同步 | 规划 |
| v1.0 | 与 `system::Autonomy` 集成，生产就绪 | 规划 |

---

## 8.11 小结

Autonomy `bridge` 模块采用 **gRPC 为主、MQTT 为辅** 的双通道架构，通过 Protobuf 强类型契约与 FSM 指令语义，在保持与内部 Navigator 解耦的同时，为外部系统提供可扩展、可观测、可安全的通信边界。当前实现处于早期阶段，Proto 与服务器框架已就绪，业务 Handler 与 MQTT 插件是后续迭代的重点。
