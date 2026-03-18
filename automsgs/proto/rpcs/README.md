# automsgs RPCs — 服务机器人第三方集成接口

本目录下所有 RPC 定义（Protocol Buffers `service` + 请求/响应消息）均直接放在 `rpcs/` 中，便于集成方按需依赖和生成客户端/服务端代码。

## 文件一览

| 文件 | 说明 | 主要 RPC |
|------|------|----------|
| `status.proto` | 公共类型，被其他 proto 引用 | `Status`, `Code` |
| `navigation.proto` | 导航 | `GoTo`, `Cancel`, `GetStatus` |
| `vehicle.proto` | 载具状态（位姿/电量/运行状态） | `GetPose`, `GetBattery`, `GetStatus` |
| `task.proto` | 任务执行 | `Run`, `GetStatus`, `Cancel` |
| `speech.proto` | 语音 | `Say` (TTS), `Listen` (ASR) |
| `charging.proto` | 充电/回桩 | `Dock`, `Undock`, `GetStatus` |
| `world.proto` | 世界（地图 + 定位） | `ListMaps`, `GetMap`, `GetMapMetadata`, `SaveMap`, `DeleteMap`, `SetCurrentMap`, `GetLocPose`, `GetLocStatus`, `SetInitialPose` |
| `sensor.proto` | 传感器 | `List`, `GetImage`, `GetLaserScan`, `GetPointCloud`, `GetGrid` |

## 包名与引用

- 包名：各文件内 `package` 仍为 `automsgs.rpcs.<模块>`（如 `automsgs.rpcs.navigation`）。
- 引用公共状态类型：`import "automsgs/rpcs/status.proto"`；引用 msgs：`import "automsgs/msgs/..."`。
- 文件名均为小写+下划线（snake_case），与 `proto/msgs`、`proto/srvs` 一致。

## 命名规范（Google Style）

本目录遵循 [Google Protocol Buffers Style Guide](https://developers.google.com/protocol-buffers/docs/style)：

| 元素 | 规范 | 示例 |
|------|------|------|
| 文件 | `lower_snake_case.proto` | `vehicle.proto`, `world.proto` |
| 包名 | 小写，多词用下划线 | `automsgs.rpcs.map_management` |
| 消息/服务/RPC | PascalCase | `GoToRequest`, `NavService`, `GoTo` |
| 字段 | snake_case | `target_pose`, `goal_id` |
| 枚举类型 | PascalCase | `NavigationState`, `Code` |
| 枚举值 | UPPER_SNAKE_CASE，且以枚举名为前缀 | `NAVIGATION_STATE_IDLE`, `CODE_OK` |
| 枚举零值 | 以 `_UNSPECIFIED` 结尾 | `NAVIGATION_STATE_UNSPECIFIED`, `CODE_UNSPECIFIED` |

## 使用方式

- **构建**：与 `msgs`/`srvs` 一起由顶层 `proto/CMakeLists.txt` 生成 C++/Python；生成产物在 `build/proto/gen/automsgs/rpcs/`。
- **传输**：当前仅生成消息与 service 描述；实际 RPC 传输可对接 gRPC 或映射到 ROS 2 service/action，由集成方实现。
- **扩展**：新增 RPC 时在 `rpcs/` 下直接添加 `.proto` 文件即可，`rpcs/*.proto` 的 GLOB 会自动包含。

## 集成建议

- 第三方只需依赖所需 proto（或生成代码），例如只做导航则仅引用 `navigation.proto` + `status.proto`。
- 所有响应均包含 `automsgs.rpcs.common.Status`（含 `code`/`message`），便于统一处理成功/错误与错误码。

## 状态码分模块设计（status.proto）

`Code` 枚举按数值区间分模块，便于扩展与判断：

| 区间 | 模块 | 示例 |
|------|------|------|
| 0-1 | 通用 | `CODE_UNSPECIFIED`, `CODE_OK` |
| 2-99 | 通用错误 | `CODE_INVALID_ARGUMENT`, `CODE_DEADLINE_EXCEEDED`, `CODE_NOT_FOUND` |
| 100-199 | 导航 | `CODE_NAVIGATION_GOAL_REJECTED`, `CODE_NAVIGATION_NO_PATH` |
| 200-299 | 载具 | `CODE_VEHICLE_LOW_BATTERY`, `CODE_VEHICLE_FAULT` |
| 300-399 | 任务 | `CODE_TASK_NOT_FOUND`, `CODE_TASK_FAILED` |
| 400-499 | 语音 | `CODE_SPEECH_SYNTHESIS_FAILED`, `CODE_SPEECH_TIMEOUT` |
| 500-599 | 充电 | `CODE_CHARGING_DOCK_NOT_FOUND`, `CODE_CHARGING_ALREADY_DOCKED` |
| 600-699 | 地图 | `CODE_MAP_NOT_FOUND`, `CODE_MAP_SAVE_FAILED` |
| 700-799 | 传感器 | `CODE_SENSOR_NOT_FOUND`, `CODE_SENSOR_UNAVAILABLE` |
| 800-899 | 定位 | `CODE_LOCALIZATION_LOST`, `CODE_LOCALIZATION_NOT_READY` |

服务端可按模块返回对应区间错误码；客户端可根据 `code` 所在区间区分模块并做统一处理。新增模块时在对应区间内追加枚举值即可。
