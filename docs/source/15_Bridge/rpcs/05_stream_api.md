(rpc-stream-api)=
# 流式接口

**什么时候用：** 需要持续进度或双向控制（遥操）时；纯查状态优先 Unary 轮询。

### 模式对照

| 模式 | Bridge 示例 | 客户端注意 |
|------|-------------|------------|
| Unary | `GetCapabilities`、`MapService/ListMaps` | 一次请求一次响应 |
| Unary → Server Stream | `Navigate`、`Follow`、`Charge Return` | 必须读 stream；拒绝/busy 可能在首帧 |
| Bidi Stream | `TeleopService/Velocity` | 用 `rpc-cli.py` 或 stub；grpcurl 不可靠 |

### 已无的全局 Stream

`ReceiveBotStates` / `ReceiveBotEvents` 一类 **全局推送已删除**。替代：

- 周期快照：`GetStatus` / `GetRobotFullInfo` 轮询  
- 任务进度：对应 Command 的 Server Stream  
- 传感器：`SensorService/GetSample`（Unary）或 `Record`（Stream）

### Server Stream 常见坑

| 坑 | 说明 |
|----|------|
| 只发 Unary 不读 stream | Navigate 接受后无 UI 进度 |
| 到达后 stream 仍开 | 导航 `ARRIVED` 按 proto **不关流**；要结束需 Cancel 或断连 |
| 断连当成功 | 客户端 disconnect 会触发服务端取消逻辑，别当 SUCCEEDED |
| busy 在 gRPC 层 vs `status` 帧 | 有的域首帧带 `*_BUSY` code，有的直接 `FAILED_PRECONDITION` |

### Bidi（遥操）

`Velocity`：START → 周期 TWIST → STOP；看门狗超时等价 STOP。  
Spin/DriveOnHeading 等仍是 Unary→Stream，与 Velocity **互斥**（`TELEOP_BUSY`）。

导航进度示例：[07 Navigation](07_navigation_command.md)。遥操：[09 Teleop](09_teleop_command.md)。
