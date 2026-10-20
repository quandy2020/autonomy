(bridge-grpc)=
# 4. gRPC

实现：`autonomy/bridge/grpc/`，基于 `autonomy/common/async_grpc`。专题见 `grpc/01`–`07`。

### 4.1 组件

| 路径 | 职责 |
|------|------|
| `grpc/DESIGN.md` | 四层 · 不变量 · 测试清单 |
| `grpc/server.*` | 生命周期、Handler 注册 |
| `grpc/handlers/*` | `automsgs.rpcs` Handler |
| `grpc/clients/*` | 域 Stub |
| `grpc/context.*` / `domains.*` | Context + DomainBundle |
| `work_scheduler` / `cancel_registry` / `task_muxer` / `state_hub` | 卸荷 · Cancel · 互斥 · 状态缓存 |

### 4.2 构建

```cpp
Server::Builder b;
b.SetServerAddress("127.0.0.1"); // 待接 GrpcOptions（host:port）
b.SetNumGrpcThreads(4);
b.SetNumEventThreads(4);
b.RegisterHandler</*…*/>();
grpc_server_ = b.Build(); // Start / Shutdown
```

| Builder | Proto | 现状 |
|---------|-------|------|
| `SetServerAddress` | `host`+`port` | 硬编码，缺 port |
| `SetNumGrpcThreads` / `SetNumEventThreads` | 对应字段 | 硬编码 4 |

接线：[grpc/05](grpc/05_bridge_integration.md) · 状态：[grpc/07](grpc/07_handlers.md)。

### 4.3 Handler

流程：`OnRequest` → `Send()`* → `Finish()`；跨线程用 `GetWriter()`。 
新增：proto → `DEFINE_HANDLER_SIGNATURE` → `RpcHandler` → `RegisterHandler`（[grpc/04](grpc/04_handler_api.md)）。

### 4.4 状态码

| Status | 场景 |
|--------|------|
| `OK` / `INVALID_ARGUMENT` / `FAILED_PRECONDITION` | 正常 / 参数 / FSM |
| `UNAVAILABLE` / `DEADLINE_EXCEEDED` | 未就绪 / 超时 |

建议：`num_event_threads` ≥ `num_grpc_threads`。

### 4.5 Navigator 对照

| Bridge | Navigator |
|--------|-----------|
| START + SINGLE_POSE | `NavigateToPose` |
| START + THROUGH_POSES | `NavigateThroughPoses` |
| REPLAN / PAUSE / RESUME | 重规划 / BT 暂停恢复 |
| CANCEL / STOP | `cancel_goal` |

### 4.6 专题索引

| 文件 | 内容 |
|------|------|
| [01](grpc/01_overview.md) | 框架定位 |
| [02](grpc/02_dual_queue.md) | CQ / EQ |
| [03](grpc/03_rpc_lifecycle.md) | Rpc 状态机 |
| [04](grpc/04_handler_api.md) | Handler API |
| [05](grpc/05_bridge_integration.md) | 接线与配置映射 |
| [06](grpc/06_upstream_reference.md) | 上游 |
| [07](grpc/07_handlers.md) | 域 Handler 表 |
