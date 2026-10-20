(bridge-architecture)=
# 2. 架构

对外契约：`automsgs.rpcs.*`。上手 [§0](00_guide.md) · API [rpcs/](rpcs/index.rst)。

### 2.1 数据流

```text
Client ──RPC──► BridgeServer ──► Handler / Stub ──► Navigator · Map
◄──响应── ◄──状态──
```

权威短文：源码旁 `autonomy/bridge/grpc/DESIGN.md`。

### 2.2 进程内四层

```text
Handler → Context / DomainBundle → Stub → Channel | Action | Cache
```

| 层 | 做 | 不做 |
|----|----|------|
| Handler | gRPC 进出、`BRIDGE_*` 宏 | 写 topic、持业务状态 |
| Context / Bundle | Muxer · Hub · Pool · CancelAll | 解析 RPC 字段 |
| Stub | 域命令、互斥、session | 碰 `async_grpc` 类型 |
| 机制 | GoalChannel / Action / SampleCache | 某一 RPC 特例 |

### 2.3 域服务

| 分类 | 服务 | 模式 | 文档 |
|------|------|------|------|
| Command | Navigation / Follow / Teleop / Charge / Exploration / Voice | Stream / Bidi | [rpcs/07+](rpcs/07_navigation_command.md) |
| Map | `MapService` | Unary | [rpcs/12](rpcs/11_map_command.md) |
| Query | Localization / Sensor / System（Info / Health / Caps） | Unary | [rpcs/04](rpcs/04_query_api.md) |
| System | Estop / ClearEstop / CancelAll | Unary | [rpcs/06](rpcs/06_system_api.md) |
