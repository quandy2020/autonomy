(bridge-grpc)=
# 4. gRPC

**实现：** `autonomy/bridge/grpc/` + `autonomy/common/async_grpc`。专题 `grpc/01`–`07`。

改 Handler、对线程/端口，或查 Status 映射。

Bridge 到 Stub/GoalChannel；BT 在 TaskServer（autonomy/task/）。

### 组件

| 路径 | 职责 |
|------|------|
| `grpc/DESIGN.md` | 四层、不变量、测试清单 |
| `grpc/server.*` | 生命周期、RegisterHandlers |
| `grpc/handlers/*` | `automsgs.rpcs` 入口 |
| `grpc/clients/*` | 域 Stub → Task / Map / … |
| `context.*` / `domains.*` | Muxer、Hub、Stub _bundle |
| `work_scheduler` / `cancel_registry` / `task_muxer` | 卸荷、Cancel、互斥 |

### 构建

需 **`BUILD_GRPC=ON`**，否则无 bridge 目标。

`BridgeServer` 构造时把 `GrpcOptions`、identity、capabilities 传入 `grpc::Server`（见 `bridge_server.cpp`）。部分 Builder 字段历史上硬编码，以当前 `server.cpp` 为准 → [grpc/05](grpc/05_bridge_integration.md)。

### Handler 流程

`OnRequest` → 校验 / muxer gate → Stub `Send` → Stream `Send`* → `Finish`。  
跨线程写 Stream 用 `GetWriter()`，勿在 Action 线程裸调 `Send`。

新增 RPC：proto → `DEFINE_HANDLER_SIGNATURE` → `RpcHandler` → `RegisterHandlers`（[grpc/04](grpc/04_handler_api.md)）。

### gRPC Status vs 业务 Status

| 层 | 典型 |
|----|------|
| gRPC | `OK`、`INVALID_ARGUMENT`、`FAILED_PRECONDITION`、`UNAVAILABLE` |
| `common.Status` | `NAVIGATION_BUSY`、`TELEOP_BUSY`、`OK` |

客户端集成应 **两层都看**。

### 与 Task 导航

| Bridge | Task/BT |
|--------|---------|
| Navigate waypoints | NavigateToPose / ThroughPoses |
| Pause/Resume/Replan/Cancel | BT Action 生命周期 |
| CancelAll | 清 muxer + 各域 Cancel |

### 专题索引

| 文件 | 内容 |
|------|------|
| [01](grpc/01_overview.md) | async_grpc 框架 |
| [02](grpc/02_dual_queue.md) | CQ / EQ |
| [03](grpc/03_rpc_lifecycle.md) | Rpc 状态机 |
| [04](grpc/04_handler_api.md) | Handler API |
| [05](grpc/05_bridge_integration.md) | BridgeServer 接线 |
| [06](grpc/06_upstream_reference.md) | 上游 async_grpc |
| [07](grpc/07_handlers.md) | 域 Handler 表 |
