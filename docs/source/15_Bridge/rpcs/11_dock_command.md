(rpc-dock-command)=
# ChargeService

源文件：`automsgs/proto/rpcs/charge.proto`  
Handler：`rpc_charge_handlers` → `ChargeStub`

自动回充 / 离桩。历史文档中的 `SendDockCommand` 已由此服务替代。

## 11.1 方法

```protobuf
service ChargeService {
  rpc Return(ReturnRequest) returns (stream ChargeResponse);
  rpc Leave(LeaveRequest) returns (stream ChargeResponse);
  rpc Pause(GoalRequest) returns (automsgs.rpcs.common.Status);
  rpc Resume(GoalRequest) returns (automsgs.rpcs.common.Status);
  rpc Cancel(GoalRequest) returns (automsgs.rpcs.common.Status);
  rpc GetStatus(GetStatusRequest) returns (ChargeResponse);
}
```

| RPC | 模式 | 说明 |
|-----|------|------|
| `Return` | Unary→Stream | 回桩；`station_id` 空=默认桩 |
| `Leave` | Unary→Stream | 离桩 |
| 生命周期 | Unary | 空 `goal_id` = 当前目标 |

## 11.2 要点

- 状态：`RETURNING` · `LEAVING` · `DOCKED_NOT_CHARGING` · `CHARGING` · `FULL` · …
- 充到满通过 `GetStatus` 观察，不是关流条件。
- 忙 → `CHARGING_BUSY`（504）。

```bash
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/charge.proto"
export SVC=automsgs.rpcs.charge.ChargeService
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/GetStatus"
```
