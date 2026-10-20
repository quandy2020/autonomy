(rpc-integration-tests)=
# 集成测试

**不是** 全量用例库；细字段以各域 proto 与 `autonomy/bridge/docs/09_cli_handbook.md` 为准。

**什么时候用：** 新环境、新镜像或改 Handler 后，用固定命令串验证 Bridge 与 task 栈是否接好。

### 前置

Bridge 已 `Start()`，task / planning / control 按你的场景已 launch。环境变量同 [01 § 环境](01_connection_guide.md)：

```bash
export REPO=/path/to/autonomy
export BRIDGE=127.0.0.1:5005
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/system.proto"
export SVC=automsgs.rpcs.system.SystemService
```

### 推荐顺序（约 5 分钟）

| 步骤 | RPC | 看什么 |
|------|-----|--------|
| 1 | `list` + `GetCapabilities` | 服务在、`bridge_version` / `supports_navigation` |
| 2 | `GetActiveGoal` | 无任务时 type 为 NONE |
| 3 | `EmergencyStop` → `ClearEmergencyStop` | `status.code` 为 OK |
| 4 | `CancelAllGoals` | 清互斥槽（联调前习惯点一次） |
| 5 | `NavigationService/Navigate` | Stream 有进度 → 终态或 ARRIVED（需地图/TF） |

```bash
grpcurl -plaintext $PROTO_OPTS -d '{}' $BRIDGE $SVC/GetCapabilities
grpcurl -plaintext $PROTO_OPTS -d '{}' $BRIDGE $SVC/GetActiveGoal
grpcurl -plaintext $PROTO_OPTS -d '{"reason":"smoke"}' $BRIDGE $SVC/EmergencyStop
grpcurl -plaintext $PROTO_OPTS -d '{}' $BRIDGE $SVC/ClearEmergencyStop
grpcurl -plaintext $PROTO_OPTS -d '{}' $BRIDGE $SVC/CancelAllGoals
```

导航与 Bidi 遥操建议 **`rpc-cli.py`**，不要强依赖 grpcurl 读长 stream。

### Stream / Unary 怎么算过

| 类型 | 结束条件 | 通过 |
|------|----------|------|
| Query / System Unary | 一条响应 | `status.code == OK`，字段合理 |
| Command Server Stream | 末帧或断流 | 末帧 `state` 为 SUCCEEDED/ARRIVED/预期 FAILED；拒绝帧看 `status` |
| Bidi Teleop | 客户端 STOP 或看门狗 | 无 `TELEOP_BUSY` 占坑时 START |

互斥：**导航进行中再 Follow** 应出现 busy 或 gRPC 层拒绝，不应两路同时占槽成功。

### Python 客户端

```bash
pip install grpcio  # 与仓库生成 stubs 版本匹配
```

Bidi、长时间 Navigate：用生成 stub 或 `rpc-cli.py`；grpcurl 对 Velocity 支持有限。

### 延伸阅读

- [01 接入](01_connection_guide.md) · [02 服务概述](02_service_overview.md)  
- 各 Command 页（[07 导航](07_navigation_command.md) 等）  
- [grpc/07 Handlers](../grpc/07_handlers.md)
