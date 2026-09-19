(rpc-follow-command)=
# FollowService

源文件：`automsgs/proto/rpcs/follow.proto`  
Handler：`rpc_follow_handlers` → `FollowStub`

## 9.1 方法

```protobuf
service FollowService {
  rpc Follow(FollowRequest) returns (stream FollowResponse);
  rpc Pause(GoalRequest) returns (automsgs.rpcs.common.Status);
  rpc Resume(GoalRequest) returns (automsgs.rpcs.common.Status);
  rpc Cancel(GoalRequest) returns (automsgs.rpcs.common.Status);
  rpc GetStatus(GetStatusRequest) returns (FollowResponse);
}
```

| RPC | 模式 | 说明 |
|-----|------|------|
| `Follow` | Unary→Stream | 跟随会话；`LOST_TARGET` 保持开流以便重捕 |
| 生命周期 | Unary | 空 `goal_id` = 当前目标 |
| `GetStatus` | Unary | 快照 |

## 9.2 要点

- `target_type`：`PERSON` / `OBJECT` / `VEHICLE` / `CUSTOM`；`target_id` 空则由实现选型。
- `FollowOptions.desired_distance_m`：期望跟距（可选）。
- 忙 → `FOLLOW_BUSY`（902）。

```bash
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/follow.proto"
export SVC=automsgs.rpcs.follow.FollowService
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/GetStatus"
```
