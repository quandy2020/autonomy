(bridge-grpc)=
# 4. gRPC 插件

gRPC 插件（`plugins/grpc/`）是 Bridge **当前唯一可用**的传输实现，基于 `autonomy/common/async_grpc` 将 [§3 RPC](03_rpc_protocol.md) 落地为异步服务端。

> **编号约定**：本页 **§4.1–§4.6** 为插件总览；`grpc/01_*`–`06_*` 为 async_grpc 专题（子页 H2 为 `{文件前缀}.x`，如 `02` → `## 2.1`）。文档地图见 [00_guide §0.1](00_guide.md#01-文档地图)。

## 4.1 组件

| 路径 | 职责 |
|------|------|
| `grpc_bridge.*` | `GrpcBridgeServer` 生命周期、Handler 注册 |
| `handlers/navigation_handler.*` | `SendNavigationCommand` |
| `handlers/exploration_handler.*` | `SendExplorationCommand` |
| `clients/navigator_stub.*` | Navigator 客户端（待实现） |
| `grpc_bridge_context.*` | `ExecutionContext` 扩展（待接线） |

## 4.2 服务器构建

```cpp
Server::Builder b;
b.SetServerAddress("127.0.0.1");  // 待读 GrpcOptions，缺 port
b.SetNumGrpcThreads(4);
b.SetNumEventThreads(4);
b.RegisterHandler<SendNavigationHandler>();
b.RegisterHandler<SendExplorationHandler>();
grpc_server_ = b.Build();  // → Start() → Shutdown()
```

| Builder 方法 | Proto 字段 | 现状 |
|--------------|-----------|------|
| `SetServerAddress` | `host` + `port` | 硬编码，缺 port |
| `SetNumGrpcThreads` | `num_grpc_threads` | 硬编码 4 |
| `SetNumEventThreads` | `num_event_threads` | 硬编码 4 |

接线缺口与目标形态：[grpc/05 §5.7](grpc/05_bridge_integration.md#57-配置-server-参数映射) · Handler 状态：[grpc/07 §7.1](grpc/07_handlers.md#71-handler-对照表)。

## 4.3 Handler

RPC 字段见 [rpcs/07 导航](rpcs/07_navigation_command.md) 等专题。流程：`OnRequest` → 多次 `Send()` → `Finish()`；跨线程写 Stream 用 `GetWriter()`。

**新增 RPC**：proto → `DEFINE_HANDLER_SIGNATURE` → `RpcHandler` → `RegisterHandler`（详见 [grpc/04 Handler API](grpc/04_handler_api.md)）。

## 4.4 状态码与调优

| gRPC Status | 场景 |
|-------------|------|
| `OK` / `INVALID_ARGUMENT` / `FAILED_PRECONDITION` | 正常 / 参数非法 / FSM 不允许 |
| `UNAVAILABLE` / `DEADLINE_EXCEEDED` | Navigator 未就绪 / 超时 |

建议：`num_event_threads` ≥ `num_grpc_threads`；大载荷调高 `max_send_message_size`。详见 [grpc/07 §7.4](grpc/07_handlers.md#74-grpc-状态码) · [grpc/02 §2.7](grpc/02_dual_queue.md#27-吞吐与线程配置)。

## 4.5 Navigator 对接

| Bridge 命令 | Navigator |
|-------------|-----------|
| `NAV_CMD_START` + `NAV_MODE_SINGLE_POSE` | `NavigateToPose` |
| `NAV_CMD_START` + `NAV_MODE_THROUGH_POSES` | `NavigateThroughPoses` |
| `NAV_CMD_REPLAN` | 重规划（保持 goals） |
| `NAV_CMD_PAUSE` / `RESUME` | BT 暂停 / 恢复 |
| `NAV_CMD_CANCEL` / `STOP` | `cancel_goal` |

## 4.6 async_grpc 专题

| 文件 | 内容 |
|------|------|
| [grpc/01](grpc/01_overview.md) | 框架定位、类图、Bridge 实例化 |
| [grpc/02](grpc/02_dual_queue.md) | CQ / EQ 双队列、线程模型 |
| [grpc/03](grpc/03_rpc_lifecycle.md) | Rpc 状态机、四种 RPC 类型 |
| [grpc/04](grpc/04_handler_api.md) | Handler API、ExecutionContext |
| [grpc/05](grpc/05_bridge_integration.md) | 源码接线、配置映射 |
| [grpc/06](grpc/06_upstream_reference.md) | cartographer async_grpc 上游 |

---

**导航**：[← §3 RPC](03_rpc_protocol.md) · [grpc/ 专题 →](grpc/index.rst) · [§5 MQTT →](05_mqtt.md)
