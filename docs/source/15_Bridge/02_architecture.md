(bridge-architecture)=
# 2. 架构

契约：`automsgs.rpcs.*`。上手 [§0](00_guide.md)；API [rpcs/](rpcs/index.rst)。设计短文：`autonomy/bridge/grpc/DESIGN.md`。

Bridge 做什么：** 把场外 gRPC 翻译成机载 Task/Map 等调用；**不算路径、不控电机**。

Bridge 到 Stub/GoalChannel；BT 在 TaskServer（autonomy/task/）。

### 数据流

```text
Client ──RPC──► BridgeServer ──► Handler → Stub ──► Task / Map / System
◄──Stream/Unary── ◄──状态──
```

TaskServer 里跑 BT；Bridge 只到 Stub/GoalChannel。

### 进程内四层

```text
Handler → Context / DomainBundle → Stub → GoalChannel | Action | Cache
```

| 层 | 做 | 不做 |
|----|----|------|
| Handler | 解析 RPC、gate、`BRIDGE_*` 宏 | 持长期业务状态、直接 pub topic |
| Context | Muxer、StateHub、WorkScheduler、Cancel | 解析 proto 业务字段 |
| Stub | 域命令、busy/estop、session | 碰 async_grpc 类型 |
| 机制 | GoalChannel、SampleCache | 单 RPC 特例逻辑堆在 Handler |

### 域服务（分类）

| 分类 | 服务 | 模式 | 文档 |
|------|------|------|------|
| Command | Navigation / Follow / Teleop / Charge / Exploration / Voice | Stream / Bidi | [rpcs/07+](rpcs/07_navigation_command.md) |
| Map | `MapService` | 多为 Unary | [rpcs/11](rpcs/11_map_command.md) |
| Query | Localization / Sensor / System | Unary | [rpcs/04](rpcs/04_query_api.md) |
| System | Estop / ClearEstop / CancelAll | Unary | [rpcs/06](rpcs/06_system_api.md) |

### 构建

`BUILD_GRPC=ON` 编进 `autonomy.bridge`；Capabilities 在 `BridgeOptions` 里配置，与 Task `enable_*` 宜一致。
