(async-grpc-bridge-integration)=
# Bridge 集成

链路：`BridgeOptions` → `BridgeServer` → `Server::Builder` → Handler。

### 5.1 启动

```cpp
Server::Builder b;
b.SetServerAddress(/* host:port from GrpcOptions */);
b.SetNumGrpcThreads(o.num_grpc_threads());
b.SetNumEventThreads(o.num_event_threads());
b.RegisterHandler</* domain handlers */>();
grpc_server_ = b.Build();
```

| 线程 | 职责 |
|------|------|
| CQ / EQ | I/O / Handler |
| WorkScheduler | Action 阻塞卸荷 |

### 5.2 配置映射

| `GrpcOptions` | Builder | 现状 |
|---------------|---------|------|
| `host`+`port` | `SetServerAddress` | 待全量接线 |
| `num_*_threads` | 对应 Setter | 部分硬编码 |
| SSL / uplink | — | 预留 |

### 5.3 Context

`Context` = Muxer · StateHub · DomainBundle · WorkScheduler。不变量见 `grpc/DESIGN.md`。

### 5.4 Checklist

- [x] 域 Handler 宏与 Stub
- [ ] `GrpcOptions` 全量传入 Builder
- [ ] `grpcurl` / `rpc-cli.py` E2E
