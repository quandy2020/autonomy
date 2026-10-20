(rpc-teleop-command)=
# TeleopService

**源文件：** `automsgs/proto/rpcs/teleop.proto`。

人工遥控速度或短动作（前进一段、后退、原地转），不走全局路径规划。先 Unary 查 Capabilities / ActiveGoal；Command 用 stream，注意 BUSY。

### RPC

| RPC | 模式 | 说明 |
|-----|------|------|
| `Velocity` | **Bidi Stream** | START → TWIST → STOP；看门狗超时 → STOP |
| `DriveOnHeading` / `BackUp` / `Spin` | Unary → Stream | 相对运动会话 |
| Pause / Resume / Cancel / GetStatus | Unary | 与导航类似 |

### 工具

| 方式 | 建议 |
|------|------|
| `rpc-cli.py` | Velocity 首选 |
| grpcurl | 仅适合 GetStatus 等 Unary |
| 自写 Bidi 客户端 | 产品遥操面板 |

### 互斥

- 任一会话进行中再开 Velocity 或 Spin → `TELEOP_BUSY`。  
- 与 Navigate/Follow **不能** 同时占互斥槽。  
- Relative 后端与 Velocity 通道有 busy 谓词联动（见 `teleop_stub`）。

### Bidi 坑

| 坑 | 说明 |
|----|------|
| 只 START 不发 TWIST | 看门狗会 STOP |
| 多客户端同时 Velocity | 第二路应 busy |
| 用 grpcurl 测 Velocity | 往往不可用，勿当回归标准 |

Stream 动作（DriveOnHeading 等）进度在对应 stream，不在 Velocity bidi 里。
