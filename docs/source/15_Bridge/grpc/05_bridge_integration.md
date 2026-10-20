(async-grpc-bridge-integration)=
# Bridge 集成

**链路：** `BridgeOptions` → `BridgeServer` → `grpc::Server` → `RegisterHandlers`。

改 `BridgeServer` 启动、端口/线程，或接入 `GrpcOptions`。

### 启动（概念）

```cpp
// bridge_server.cpp：构造时传入 GrpcOptions + identity + capabilities
grpc_bridge_ = std::make_unique<grpc::Server>(
    options_.grpc(), options_.identity(), options_.capabilities());
grpc_bridge_->Start();
```

Lua / `bridge.pb.txt` 经 `CreateOptions` 填 `BridgeOptions`；字段见 [01 参数](../01_options.md)。

### 线程

| 池 | 配置字段 | 职责 |
|----|----------|------|
| CQ | `num_grpc_threads` | 网络 I/O |
| EQ | `num_event_threads` | Handler 逻辑 |
| WorkScheduler | `num_worker_threads`（0=自动） | 阻塞 Action 卸荷 |

建议 `num_event_threads` ≥ `num_grpc_threads`。

### Context

运行期共享：`TaskMuxer`、`StateHub`、`DomainBundle`（各 Stub）、`CancelRegistry`。  
不变量与测试清单：`autonomy/bridge/grpc/DESIGN.md`。

### 构建开关

- 根 CMake **`BUILD_GRPC=ON`**（默认）才编 `autonomy.bridge`。  
- 关则无 gRPC 端口；`Capabilities` 再全 true 也连不上。

### 接线现状（写文档时请以代码为准）

部分 `Server::Builder` 仍可能硬编码线程数或地址；以 `grpc/server.cpp` 与 [04 gRPC](../04_grpc.md) 为准。联调以 `grpcurl GetCapabilities` 为金标准。

### 常见坑

| 现象 | 说明 |
|------|------|
| conf 改 port 不生效 | Builder 未接全 GrpcOptions |
| Handler 在、Stub 未接 Autolink | 命令 ACK 但下游无动作 |
| 关 BUILD_GRPC 后找 bridge 二进制 | 目标未生成 |
