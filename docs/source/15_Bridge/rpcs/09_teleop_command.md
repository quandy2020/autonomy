(rpc-teleop-command)=
# TeleopService

源：`teleop.proto`。

| RPC | 模式 |
|-----|------|
| `Velocity` | Bidi（START/TWIST/STOP；看门狗超时 STOP） |
| DriveOnHeading / BackUp / Spin | Unary→Stream |
| 生命周期 / GetStatus | Unary |

Bidi 请用 `rpc-cli.py`（grpcurl 支持有限）。忙 → `TELEOP_BUSY`。
