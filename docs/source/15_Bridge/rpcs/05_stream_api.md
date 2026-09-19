(rpc-stream-api)=
# 流式接口 Stream

`automsgs.rpcs` **不再**提供 `ReceiveBotStates` / `ReceiveBotEvents`（已随 AutonomyService 移除）。

状态与事件请用：

| 需求 | 推荐 |
|------|------|
| 周期粗状态 | `SystemService/GetStatus` 轮询，或订阅机载 `/robot_state` |
| 完整画像 | `SystemService/GetRobotFullInfo` |
| 命令进度 | 各域 **Unary→Stream**（`Navigate` / `Follow` / `Return` / …） |
| 遥操速度 | `TeleopService/Velocity` **Bidi** |
| 传感器录制 | `SensorService/Record` Stream |

## 5.1 命令流 vs 推送

```text
历史全局推送（已移除）：Empty → stream RobotState
现 automsgs.rpcs：      按域命令流携带进度；系统状态 Unary 查询
```

Bridge 侧 `StateHub` 仅缓存 `/robot_state` 供 System 查询合成，**不**再对外 push fan-out。

## 5.2 示例：导航进度流

见 [07 NavigationService](07_navigation_command.md)。
