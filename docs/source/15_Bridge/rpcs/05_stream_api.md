(rpc-stream-api)=
# 流式接口

全局 `ReceiveBotStates/Events` 已移除。

| 需求 | 方式 |
|------|------|
| 周期状态 | `GetStatus` / `GetRobotFullInfo` 轮询 |
| 命令进度 | 域 Unary→Stream（Navigate 等） |
| 遥操 | `TeleopService/Velocity` Bidi |
| 录制 | `SensorService/Record` |

导航进度示例见 [07](07_navigation_command.md)。
