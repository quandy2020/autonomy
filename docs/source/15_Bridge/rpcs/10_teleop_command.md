(rpc-teleop-command)=
# TeleopService

源文件：`automsgs/proto/rpcs/teleop.proto`  
Handler：`rpc_teleop_handlers` → `TeleopStub`

## 10.1 方法

```protobuf
service TeleopService {
  rpc Velocity(stream VelocityRequest) returns (stream TeleopResponse);
  rpc DriveOnHeading(DriveOnHeadingRequest) returns (stream TeleopResponse);
  rpc BackUp(BackUpRequest) returns (stream TeleopResponse);
  rpc Spin(SpinRequest) returns (stream TeleopResponse);
  rpc Pause(GoalRequest) returns (automsgs.rpcs.common.Status);
  rpc Resume(GoalRequest) returns (automsgs.rpcs.common.Status);
  rpc Cancel(GoalRequest) returns (automsgs.rpcs.common.Status);
  rpc GetStatus(GetStatusRequest) returns (TeleopResponse);
}
```

| RPC | 模式 | 说明 |
|-----|------|------|
| `Velocity` | Bidi | `START` / `TWIST` / `STOP`；看门狗超时自动 STOP |
| `DriveOnHeading` / `BackUp` / `Spin` | Unary→Stream | 相对运动；`Pause` 不关流 |
| 生命周期 | Unary | 作用于当前 Velocity 或相对目标 |

## 10.2 要点

- 同一时刻至多一个遥操目标；忙 → `TELEOP_BUSY`。
- `VelocityOptions.watchdog_timeout_seconds`：无 TWIST 则自动 STOP。
- 相对运动走 Action CRTP（同属 `TeleopStub`）。

推荐用 `rpc-cli.py` 做 Bidi；grpcurl 对 Bidi 支持有限。
