(async-grpc-overview)=
# async_grpc

路径：`autonomy/common/async_grpc/`。上游：[cartographer async_grpc](https://github.com/cartographer-project/async_grpc)。消费方：`autonomy/bridge/grpc`。

固定 CQ/EQ 线程池 Multiplex 连接；`RpcHandler` 抽象替代手写异步 tag 状态机。

| 目标 | 说明 |
|------|------|
| 双队列 | CQ=I/O；EQ=Handler（[02](02_dual_queue.md)） |
| 单 Rpc 单线程 | Handler 不必 thread-safe |
| 四种 RPC | `Stream<T>`（[03](03_rpc_lifecycle.md)） |
| 跨线程写 | `GetWriter()` + `WRITE_NEEDED` |
| 共享上下文 | `ExecutionContext` |

### 1.2 组件

| 类 | 职责 |
|----|------|
| `Server` | 线程池、注册表、Context |
| `Service` / `Rpc` | 路由与单次状态机 |
| `RpcHandler` | `OnRequest` / `Send` / `GetWriter` |

```{figure} images/diagram.png
:alt: async_grpc 架构
:align: center
:width: 88%
:name: fig-async-grpc-architecture
```

### 1.3 默认常量

| 常量 | 值 |
|------|-----|
| `kDefaultMaxMessageSize` | 10 MB |
| Bridge `kMaxMessageSize` | 100 MB |
| `kPopEventTimeout` | 100 ms |
