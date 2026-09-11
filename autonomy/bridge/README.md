# autonomy/bridge

机载 gRPC 对外接口。默认监听 `127.0.0.1:5005`（见 `config/bridge/`）。

## 部署资产

| 路径 | 用途 |
|------|------|
| `launch/bridge.launch` | 独立启动 `autonomy.bridge`（respawn） |
| `conf/bridge.pb.txt` | BridgeOptions 默认值（文档 / 对齐 Lua） |
| `dag/bridge.dag` | Autolink 通道拓扑说明（**非** mainboard DAG） |
| `config/bridge/*.lua` | 运行时入口（`bridge_options.lua`） |

```bash
# 需 -DBUILD_GRPC=ON
export PATH=$PWD/build/bin:$PATH
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/bridge/launch
autolink_launch bridge.launch
```

## 双表面

| 表面 | 包名 | 用途 |
|------|------|------|
| **AutonomyService** | `autonomy.bridge.proto` | Bridge 客户端（CommandAck 流） |
| **automsgs.rpcs.\*** | `automsgs.rpcs.*` | 第三方 / `rpc-cli.py` |

共用 Stub → Autolink / Action，并由 `TaskMuxer` 互斥。

## 业务覆盖

| 业务 | AutonomyService | automsgs.rpcs | 下游 |
|------|-----------------|---------------|------|
| 单点/多点导航 | `SendNavigationCommand` | `NavigationService` | `/navigate_to_pose` · through_poses |
| 人体跟随 | `SendFollowCommand` | `FollowService` | `/autonomy/task/tracking/*` |
| 遥操 Velocity | `SendTeleopCommand` | `TeleopService/Velocity` | `/autonomy/task/teleop/*` |
| 相对运动 | — | `DriveOnHeading` / `BackUp` / `Spin` | `/drive_on_heading` `/backup` `/spin` |
| 自动回充 | `SendDockCommand` | `ChargeService` | `/autonomy/task/charging/*` |
| 地图管理 | `SendMapCommand` | `MapService` | mapping goal + `/map` 缓存 |
| 探索建图 | `SendExplorationCommand` | `ExplorationService` | mapping + `/exploration/*` |
| 定位 | — | `LocalizationService` | `/amcl_pose` + Map SET_INITIAL_POSE |
| 语音控制 | `SendVoiceCommand` | `VoiceService` | 意图分发 |
| 传感器 | — | `SensorService` | 参数 KV + RecordWriter 录制 |
| 状态推送 | `ReceiveBotStates/Events` · `GetRobotSnapshot` | `SystemService` | `/robot_state` `/robot_event` |
| 系统健康 | Estop / CancelAll / Capabilities | Heartbeat / GetInfo / GetStatus / **GetHealth** | 内嵌 `MonitorRegistry` |

## 关键代码

- `CMakeLists.txt` — `autonomy.bridge` 二进制 + conf/dag/launch 安装
- `grpc/grpc_bridge.cpp` — Handler 注册（显式 `RegisterHandler`）
- `grpc/handlers/` — **按任务域合并**（同域多 Handler 一类文件）：`command_handlers.*`、`rpc_<domain>_handlers.*`；`rpc_handlers.hpp` 聚合 include
- `grpc/handlers/handler_util.hpp` — Unary / Stream 模板（无 class）
- `grpc/clients/goal_channel_stub.hpp` — **Traits** 会话基座（Follow / Dock / Map / Teleop）
- `grpc/clients/action_goal_session.hpp` — Action `SendActionGoal` 模板（Navigator）
- `grpc/clients/stream_session.hpp` — 多通道流会话状态（Exploration）
- `grpc/clients/latest_message_cache.hpp` — 最新消息缓存（Localization / MapService）
- `grpc/clients/sensor_sample_traits.hpp` — 传感器样本类型萃取 + `SubscribeSample`
- `grpc/clients/stub_util.hpp` — `FillCommandAck` / `DispatchCommands` / `MakeCommandRule(s)` / Reject 守卫
- `grpc/clients/*_stub.*` — 域 Stub（一文件一类）
- 命名约定：函数 **动词 + 名词**；变量/参数不缩写；保留专业名词（Estop、IMU、CommandAck、muxer）；公开 API 使用 Doxygen
- `grpc/state_hub.*` — 状态缓存与推送
- `proto/external_command_service.proto` — AutonomyService 契约
