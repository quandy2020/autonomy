(async-grpc-overview)=
# async_grpc

**路径：** `autonomy/common/async_grpc/`（上游 Cartographer async_grpc）。消费方：`autonomy/bridge/grpc`。
| 设计点 | 说明 |
|--------|------|
| 双队列 | CQ 做 I/O，EQ 跑 Handler → [02](02_dual_queue.md) |
| 单 Rpc 单线程 | Handler 实例不必 thread-safe |
| 四种 RPC | Unary / Server stream / Client stream / Bidi → [03](03_rpc_lifecycle.md) |
| 跨线程写 | `GetWriter()` + `WRITE_NEEDED` |
| 共享状态 | `ExecutionContext` / Bridge `Context` |

是什么：** Bridge 用的 gRPC 服务端框架，避免手写 completion-queue tag 状态机。

### 核心类

| 类 | 职责 |
|----|------|
| `Server` | 线程池、Handler 注册 |
| `Rpc` | 单次调用状态机 |
| `RpcHandler` | `OnRequest` / `Send` / `Finish` |

```{figure} images/diagram.png
:alt: async_grpc 架构
:align: center
:width: 88%
:name: fig-async-grpc-architecture
```

### 消息大小

Bridge 侧上限约 **100 MB**（大点云/地图片段）；默认 async_grpc 约 10 MB。超大 payload 仍应走专用通道或分片。
