(rpc-navigation-command)=
# NavigationService

**契约：** `automsgs/proto/rpcs/navigation.proto`；实现经 `NavigatorStub` → Task 侧 GoalChannel。

在需要单点或多航点导航，且上游走 Bridge 而不是进程内 Action

- **应开：** 需要进度、剩余距离、航点索引时，客户端应挂住 `Navigate` 的 stream，直到收到 `FAILED` / `CANCELLED` 或业务拒绝帧，或主动 `Cancel`。  
- **不必开：** 只探空闲态时用 `GetStatus` 或 `SystemService/GetActiveGoal`。

Proto 约定：**到达末航点 `ARRIVED` 后 stream 仍可保持打开**（便于 Pause/Replan）；终态失败或 Cancel 后 stream 会结束。不要假设「SUCCESS 即关流」。

### RPC 一览

| RPC | 模式 | 用途 |
|-----|------|------|
| `Navigate` | Unary → **Server Stream** | 下发 waypoints，收进度直到终态或断流 |
| `Pause` / `Resume` / `Replan` / `Cancel` | Unary | 生命周期；空 `goal_id` = 当前目标 |
| `GetStatus` | Unary | 快照，字段与流中 `NavigateResponse` 同形 |

### 互斥与 busy

- 同时最多 **一个** 导航类互斥目标（`TaskMuxer`）。  
- 已有 PLANNING/RUNNING/PAUSED/ARRIVED 时再 `Navigate` → `Status.code = NAVIGATION_BUSY`（105），**不会**挤掉旧目标。  
- 要先换任务：`Cancel` 或 `SystemService/CancelAllGoals`，再发新 `Navigate`。  
- `EmergencyStop` 可抢占；清急停后再导航。

### Unary 生命周期 RPC 的坑

`Pause` / `Resume` / `Replan` / `Cancel` **只返回 `common.Status`**，进度仍在 Navigate stream 或 `GetStatus` 里。客户端若只调 Pause 不读 stream，会以为「没反应」。

### grpcurl 示例

```bash
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/navigation.proto"
export SVC=automsgs.rpcs.navigation.NavigationService
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/GetStatus"
```

带 waypoints 的 `Navigate` 载荷字段见 proto；多点时 `waypoints` 长度 > 1，中间点只推进 `waypoint_index`，末点到达才 `ARRIVED`。

### 常见坑

| 现象 | 原因 |
|------|------|
| `NAVIGATION_BUSY` | 上一导航未 Cancel；或 Follow/Teleop 等占互斥槽 |
| Stream 无帧 | 未读 stream；或首帧就是拒绝（看 `status.message`） |
| `ARRIVED` 但 stream 不断 | 符合 proto；要结束需 Cancel 或断开客户端 |
| INVALID_ARGUMENT | waypoints 空、header/frame 不对 |

对照 [02 服务概述](02_service_overview.md)、[grpc/07 Handlers](../grpc/07_handlers.md)。
