(rpc-navigation-command)=
# NavigationService

源文件：`automsgs/proto/rpcs/navigation.proto`  
Handler：`rpc_navigation_handlers` → `NavigatorStub`

Unary 请求 → Server Stream（`Navigate`）；生命周期为 Unary（`Pause` / `Resume` / `Replan` / `Cancel` / `GetStatus`）。

## 7.1 方法

```protobuf
service NavigationService {
  rpc Navigate(NavigateRequest) returns (stream NavigateResponse);
  rpc Pause(GoalRequest) returns (automsgs.rpcs.common.Status);
  rpc Resume(GoalRequest) returns (automsgs.rpcs.common.Status);
  rpc Replan(GoalRequest) returns (automsgs.rpcs.common.Status);
  rpc Cancel(GoalRequest) returns (automsgs.rpcs.common.Status);
  rpc GetStatus(GetStatusRequest) returns (NavigateResponse);
}
```

| RPC | 模式 | 说明 |
|-----|------|------|
| `Navigate` | Unary→Stream | `waypoints` 长度 1=单点，N=多点；末点 `ARRIVED` 不关流 |
| `Pause` / `Resume` / `Replan` / `Cancel` | Unary | 空 `goal_id` = 当前目标 |
| `GetStatus` | Unary | 返回与流帧同形的 `NavigateResponse` |

## 7.2 请求 / 响应要点

- `NavigateRequest.goal_id`：客户端幂等键；空则由实现生成。
- `NavigateOptions`：线/角速度上限、到位容差、超时（可选）。
- `NavigateResponse.state`：`IDLE` · `PLANNING` · `RUNNING` · `PAUSED` · `ARRIVED` · `FAILED` · `CANCELLED`。
- 忙冲突 → `NAVIGATION_BUSY`（105）。同一时刻至多一个导航目标。

## 7.3 grpcurl 示例

```bash
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/navigation.proto"
export SVC=automsgs.rpcs.navigation.NavigationService

grpcurl -plaintext $PROTO_OPTS -d @- "$BRIDGE" "$SVC/GetStatus" <<'EOF'
{}
EOF

grpcurl -plaintext $PROTO_OPTS -d @- "$BRIDGE" "$SVC/Cancel" <<'EOF'
{ "goal_id": "" }
EOF
```

完整字段以 proto 为准；接入流程见 [01](01_connection_guide.md)。
