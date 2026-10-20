(rpc-follow-command)=
# FollowService

源：`follow.proto`。

| RPC | 模式 |
|-----|------|
| `Follow` | Unary→Stream（`LOST_TARGET` 可保持开流） |
| Pause/Resume/Cancel/GetStatus | Unary |

要点：`target_type`（PERSON/OBJECT/…）；`desired_distance_m`；忙 → `FOLLOW_BUSY`。

```bash
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/follow.proto"
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" automsgs.rpcs.follow.FollowService/GetStatus
```
