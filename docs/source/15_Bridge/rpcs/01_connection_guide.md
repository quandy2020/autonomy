(bridge-connection-guide)=
# 接入与验证

**前提：** Bridge 进程已起，且编译时 `-DBUILD_GRPC=ON`（否则没有 `autonomy.bridge` 域）。

**什么时候用：** 场外 App、联调脚本或 CI 需要连机载 gRPC 时，先走本页再调具体 Command。

### 环境变量（建议固定一套）

| 项 | 典型值 | 说明 |
|----|--------|------|
| 地址 | `127.0.0.1:5005` | 与 [§1](../01_options.md) 里 `GrpcOptions` 一致 |
| Proto 根 | 仓库根 `$REPO` | `grpcurl` 的 `-import-path` |
| 系统服务 | `automsgs.rpcs.system.SystemService` | 冒烟首选 |

```bash
export REPO=/path/to/autonomy
export BRIDGE=127.0.0.1:5005
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/system.proto"
export SVC=automsgs.rpcs.system.SystemService
```

换域时只改 `-proto` 与 `$SVC`（例如导航用 `automsgs/proto/rpcs/navigation.proto`）。

### 服务发现

```bash
grpcurl -plaintext $PROTO_OPTS $BRIDGE list
grpcurl -plaintext $PROTO_OPTS $BRIDGE list $SVC
grpcurl -plaintext $PROTO_OPTS $BRIDGE describe $SVC.GetCapabilities
```

| 结果 | 含义 |
|------|------|
| `list` 含 `automsgs.rpcs.*.*Service` | 反射或静态注册正常 |
| `describe` 能展开请求/响应 | proto 路径无误 |
| 连接 refused | 端口错、Bridge 未 Start、或 `BUILD_GRPC=OFF` 未编进镜像 |

### 冒烟（建议顺序）

1. `GetCapabilities` — 版本与 `supports_*`  
2. `GetActiveGoal` — 互斥槽是否空闲  
3. （可选）`GetRobotFullInfo` — 身份 + 状态一次看完  

```bash
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/GetCapabilities"
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/GetActiveGoal"
```

### 工具选型

| 工具 | 适合 | 不适合 |
|------|------|--------|
| `grpcurl` | Unary、Server Streaming（Navigate 等） | Bidi `TeleopService/Velocity` |
| `automsgs/tools/cli/rpc-cli.py` | 全 RPC，含 Bidi、读 Stream 到终态 | 无 Python/grpcio 环境 |
| 生成 stubs + 自写客户端 | 产品集成 | 快速手工探测 |

### 常见坑

| 现象 | 先查 |
|------|------|
| `Failed to dial` | `host:port`；防火墙；容器是否映射 5005 |
| `unknown service` | 旧二进制；Capabilities 里对应 `supports_*` 是否为 false |
| proto import 失败 | `-import-path` 必须是含 `automsgs/` 的仓库根 |
| RPC OK 但车不动 | Task / planning / control 进程；见 [17 Tasks](../../17_Tasks/00_guide.md) |

用例串见 [12 集成测试](12_integration_tests.md)。
