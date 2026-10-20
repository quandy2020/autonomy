(rpc-follow-command)=
# FollowService

**源文件：** `automsgs/proto/rpcs/follow.proto`。

在跟随行人/目标，而不是地图航点导航。与 `NavigationService` **互斥**（同一 muxer 槽）

- `target_type`（人/物等）、`desired_distance_m` 等见 proto。  
- 目标丢失时 stream 可能保持并上报 `LOST_TARGET` 类状态（以便 UI 提示），不一定立即关流。  
- 导航进行中调 `Follow` → `FOLLOW_BUSY` 或 gate 拒绝；先 `CancelAllGoals` 或 Cancel 导航。

```bash
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/follow.proto"
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" automsgs.rpcs.follow.FollowService/GetStatus
```

### RPC

| RPC | 模式 |
|-----|------|
| `Follow` | Unary → Server Stream |
| `Pause` / `Resume` / `Cancel` / `GetStatus` | Unary |

### Stream vs Unary

生命周期与导航类似：`Pause` 只回 Status，进度在 `Follow` stream 或 `GetStatus`。

### 常见坑

| 现象 | 说明 |
|------|------|
| `FOLLOW_BUSY` | Teleop/Nav/Charge 等占槽 |
| 跟上了但 stream 无帧 | 未读 stream；感知/跟踪节点未起 |
| 与 Navigate 同时「成功」 | 不应出现；查 muxer 测试 |
