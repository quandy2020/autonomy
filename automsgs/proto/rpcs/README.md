# automsgs RPCs — 服务机器人第三方集成接口

本目录下所有 RPC（Protocol Buffers `service` + 请求/响应）按机器人领域划分，便于按需依赖与生成客户端/服务端代码。

## 文件一览

| 文件 | package | Service | 说明 |
|------|---------|---------|------|
| `common.proto` | `automsgs.rpcs.common` | — | `Status`（`code` = `status_msgs.StatusCode`） |
| `navigation.proto` | `automsgs.rpcs.navigation` | `NavigationService` | 导航：Navigate 流式 + Pause/Resume/Cancel/GetStatus；位姿/ETA/BUSY |
| `mapping.proto` | `automsgs.rpcs.mapping` | `MapService` | StartMapping/FinishMapping/CancelMapping + 地图存储 |
| `localization.proto` | `automsgs.rpcs.localization` | `LocalizationService` | 定位 / 重定位 |
| `charge.proto` | `automsgs.rpcs.charge` | `ChargeService` | 自动回充：Return/Leave 流式 + Pause/Resume/Cancel/GetStatus |
| `follow.proto` | `automsgs.rpcs.follow` | `FollowService` | 跟随：Follow 流式 + Pause/Resume/Cancel/GetStatus |
| `sensor.proto` | `automsgs.rpcs.sensor` | `SensorService` | 列举、快照、kv 参数持久化、录制/上传 |
| `system.proto` | `automsgs.rpcs.system` | `SystemService` | Heartbeat、状态、E-Stop/Clear、CancelAll、ActiveGoal、Capabilities |
| `teleop.proto` | `automsgs.rpcs.teleop` | `TeleopService` | 遥控：Velocity + DriveOnHeading/BackUp/Spin + Pause/Resume |
| `exploration.proto` | `automsgs.rpcs.exploration` | `ExplorationService` | 自主探索建图：Explore 流式 + Pause/Resume/Cancel/SetArea/SaveMap |

## 包名与引用

- 公共状态：`import "automsgs/rpcs/common.proto"`。
- 消息载荷：`import "automsgs/msgs/..."`（优先复用既有 msgs）。
- 文件名 `lower_snake_case.proto`，与 `proto/msgs` 一致。

## 命名规范（Google Style）

| 元素 | 规范 | 示例 |
|------|------|------|
| 文件 | `lower_snake_case.proto` | `charge.proto` |
| 包名 | 小写 | `automsgs.rpcs.charge` |
| 消息/服务/RPC | PascalCase | `ReturnRequest`, `ChargeService`, `Return` |
| 字段 | snake_case | `goal_id`, `station_id` |
| 枚举类型 | PascalCase | `ChargeState`, `StatusCode` |
| 枚举值 | UPPER_SNAKE | `CHARGE_STATE_IDLE`, `OK`, `NAVIGATION_BUSY` |
| 枚举零值 | 域状态 `_UNKNOWN`；**Status 成功为 `OK=0`** | `CHARGE_STATE_UNKNOWN` / `OK` |

## 长任务约定

建图等长任务（默认 unary 接受 + 轮询）：

1. `StartMapping` 等 **立即返回** `Status`（接受会话；进度 `GetMappingStatus`）。
2. 进度与终态通过 `GetStatus`（或域内等价查询）轮询。
3. `Cancel(goal_id)`：空 id = 取消当前活动目标。
4. 单一活动目标；忙时拒绝新请求（对应模块 `*_BUSY` 或域内拒绝码），不抢占。

### Navigation（流式例外）

| RPC | 形态 | 说明 |
|-----|------|------|
| `Navigate` | **server streaming** | `waypoints`；`stream NavigateResponse`（位姿/ETA） |
| `Pause` / `Resume` / `Replan` / `Cancel` | **unary** | 共用 `GoalRequest`；只回 `Status`；进度看流或 `GetStatus` |
| `GetStatus` | **unary** | 直接返回 `NavigateResponse` |

**航点：** 中间点 `PLANNING`/`RUNNING`；终到 `ARRIVED`。空 `waypoints` → `INVALID_ARGUMENT`。

**Options：** `maximum_linear_speed` / `maximum_angular_speed` / 到位容差 / `timeout_seconds`。

**状态：** `UNKNOWN`/`IDLE`/`PLANNING`/`RUNNING`/`PAUSED`/`ARRIVED`/`FAILED`/`CANCELLED`。占用目标：PLANNING–ARRIVED。忙 → `NAVIGATION_BUSY`（105）。

**关流：** Cancel / `FAILED` / `CANCELLED` / 拒收 / 客户端断开。**`ARRIVED` 不关流。**

### Charge（流式例外）

| RPC | 形态 | 说明 |
|-----|------|------|
| `Return` / `Leave` | **server streaming** | `stream ChargeResponse`（`state` / `progress`） |
| `Pause` / `Resume` / `Cancel` | **unary** | 共用 `GoalRequest`；只回 `Status` |
| `GetStatus` | **unary** | 直接返回 `ChargeResponse` |

**关流：** 回充/离桩终态、Cancel、FAILED、断开。**`PAUSED` 不关流。** 不以充满电关流；驻留用电量用 `GetStatus`。

忙 → `CHARGING_BUSY`（504）。`Cancel` → 流末条 `CANCELLED` + `CHARGING_CANCELLED`（505）。

### Follow（流式例外）

| RPC | 形态 | 说明 |
|-----|------|------|
| `Follow` | **server streaming** | `stream FollowResponse`（`state` / `distance_m`）；会话期保持开流 |
| `Pause` / `Resume` / `Cancel` | **unary** | 共用 `GoalRequest`；只回 `Status` |
| `GetStatus` | **unary** | 直接返回 `FollowResponse` |

**目标：** `FollowTargetType` + `target_id`（空=实现自选）+ 可选 `hint_pose` / `FollowOptions`。

**关流：** Cancel / `FAILED` / 会话结束 / 断开。**`LOST_TARGET` / `PAUSED` 不关流。**

忙 → `FOLLOW_BUSY`（902）。无 `FollowPhase`（与 Charge 一致，只用粗状态）。

### Sensor

| RPC | 形态 | 说明 |
|-----|------|------|
| `ListSensors` / `GetSample` | **unary** | 目录；最新帧 `oneof`（Image / Compressed / Scan / Cloud / Imu） |
| `GetParameters` / `SetParameters` | **unary** | 键值 `name`/`value`；`names` 空 = 全部 |
| `SaveParameters` / `LoadParameters` | **unary** | 机器人侧配置持久化；`sensor_id` 空 = 全部 |
| `Record` | **server streaming** | 录制/导出/上传；含 `active` / `final` |
| `CancelRecord` / `GetRecordStatus` | **unary** | `GetRecordStatus.record` 嵌入 `RecordResponse` |

**删除：** 按类型 `GetImage` / `GetLaserScan` / `GetPointCloud` / `GetOccupancyGrid`（栅格见 `mapping`）。

**关流：** 成功 / `FAILED` / Cancel / 忙拒 → `final=true`。忙时新 Record → `SENSOR_BUSY`（703）。

### System

| RPC | 形态 | 说明 |
|-----|------|------|
| `Heartbeat` | **unary** | 保活；回显 `sequence` + `robot_time_ns` |
| `GetInfo` | **unary** | 型号 / 序列号 / 软硬件版本 / 主机名 |
| `GetStatus` | **unary** | `SystemState` + `battery` + `SystemHealth`（monitor 摘要） |
| `GetHealth` | **unary** | 完整 `SystemHealth`（Hazard/MRM/主机/通道/时延） |
| `EmergencyStop` / `ClearEmergencyStop` | **unary** | 急停 / 解除（可要求 `confirmation_token`） |
| `CancelAllGoals` | **unary** | 取消活动域目标；`goal_kinds` 空 = 全部 |
| `GetActiveGoal` | **unary** | 当前占用运动/建图的域目标 |
| `GetCapabilities` | **unary** | 能力发现（含 `supports_system_monitor` / `supports_mrm`） |

**与 `autonomy/system/monitor`：** `HazardLevel`、`ChannelHealth`、`LatencyHealth`、MRM 与主机资源对齐；见 monitor README。

### Teleop

| RPC | 形态 | 说明 |
|-----|------|------|
| `Velocity` | **bidi streaming** | 持续 `cmd_vel`：START/STOP/TWIST；选项 `VelocityOptions`（含 watchdog） |
| `DriveOnHeading` | **server streaming** | 沿朝向平移（Nav2）；优先 `distance_meters >= 0` |
| `BackUp` | **server streaming** | 沿朝向后退（Nav2）；`distance_meters > 0` |
| `Spin` | **server streaming** | 相对偏航（Nav2）；`target_yaw_radians` 正=逆时针 |
| `Pause` / `Resume` | **unary** | 仅相对运动；`PAUSED` **不关流**；Velocity 用 STOP |
| `Cancel` / `GetStatus` | **unary** | → `TeleopResponse` |

**Options：** `VelocityOptions` vs `RelativeOptions`（容差 / timeout，无 watchdog）。  
**反馈：** `current_pose`、`commanded_twist`、`actual_twist`、剩余距离/偏航。  
**状态：** `ACTIVE` / `PAUSED` / `SUCCEEDED` / `FAILED` / `TIMEOUT` / `REJECTED` / `CANCELLED`。

**关流：** Velocity：`STOP` / Cancel / TIMEOUT / REJECTED / FAILED / 断开。相对：`SUCCEEDED` / FAILED / Cancel / TIMEOUT / 断开（**Pause 不关流**）。

忙 → `TELEOP_BUSY`(1200)；碰撞 → `1204`；未就绪 → `1205`。

### Exploration（流式例外）

| RPC | 形态 | 说明 |
|-----|------|------|
| `Explore` | **server streaming** | `area` + `map_name` + `ExploreOptions`；`stream ExploreResponse` |
| `Pause` / `Resume` / `Cancel` | **unary** | 共用 `GoalRequest`；只回 `Status` |
| `GetStatus` | **unary** | 直接返回 `ExploreResponse` |
| `SetArea` | **unary** | 运行中更新探索多边形（可触发重规划） |
| `SaveMap` | **unary** | 会话内存图；列表/加载仍走 `MapService` |

**关流：** COMPLETED / FAILED / CANCELLED / 拒收 / 断开。**`PAUSED` 不关流。**

忙 → `EXPLORATION_BUSY`（1300）。地图 CRUD 不在本服务。

## 状态码（统一 `status_msgs.StatusCode`）

`Status.code` 类型为 `automsgs.msgs.status_msgs.StatusCode`（与内部 `StatusPb` 共用码表）。  
**成功：`OK = 0`（breaking：旧 RPC `CODE_OK = 1`）。未知：`UNKNOWN = 1`。**  
系统域为避开 CONTROL 1000 段，使用 **2000–2099**。细节见 `2026-08-15-status-code-unify-design.md`。

**命名：** 不加全局 `CODE_` / `RPC_`；用域前缀（`NAVIGATION_*`、`TASK_*`、`MAP_*`、`LOCALIZATION_*`）。同名冲突时换语义名并分数字段（如 RPC `LOCALIZATION_UNAVAILABLE=801` vs 模块 `LOCALIZATION_NOT_READY=3002`）。

**分层：** `Status.code` 只使用**薄码**（0–999 中的 RPC 域、1100–1399、2000–2099 及通用码）。  
`CONTROL_*` / `PLANNING_*` / `TASK_*`（9000+）等为**细码**，供内部 `StatusPb`；第三方勿依赖。  
取消拼写统一为 `CANCELLED`；BT 任务取消细码为 `TASK_BT_CANCELLED`（9003）。  
结构与去别名阶段见 `docs/superpowers/specs/2026-08-15-status-code-structure-design.md`。  
出站若持有细码，用 `automsgs/msgs/status_msgs/status_code_map.hpp` 的 `ToThinStatusCode` 再写入 `Status.code`。

| 区间 | 层 | 模块 |
|------|----|------|
| 0–1 | 薄 | `OK` / `UNKNOWN` |
| 2–99 | 薄 | 通用错误（`INVALID_ARGUMENT` 等） |
| 100–199 | 薄 | 导航 |
| 200–299 | 薄 | 载具 |
| 300–399 | 薄 | 任务面（`TASK_*`；与 9000+ 行为树 TASK 分段） |
| 400–499 | 薄 | 语音 |
| 500–599 | 薄 | 回充（`CHARGING_*`） |
| 600–699 | 薄 | 地图存储（`MAP_*`；与 7000+ 模块 MAP 分段） |
| 700–799 | 薄 | 传感器 |
| 800–899 | 薄 | 定位 RPC（`LOCALIZATION_BUSY` 等；`LOCALIZATION_UNAVAILABLE=801`） |
| 900–999 | 薄 | 跟随 |
| 1000–1099 | 细 | `CONTROL_*` |
| 1100–1199 | 薄 | 建图过程 |
| 1200–1299 | 薄 | 遥操 |
| 1300–1399 | 薄 | 探索 |
| 2000–2099 | 薄 | System（`SYSTEM_ESTOP` 等） |
| 3000+ | 细 | localization / recovery / planning / … |

## 使用方式

- **构建**：与 `msgs`/`srvs` 由 `proto/CMakeLists.txt` GLOB `rpcs/*.proto` 生成。
- **传输**：仅定义 service/消息；可由 gRPC 或映射到 ROS 2 service/action 实现。
- **扩展**：在 `rpcs/` 新增 `.proto` 即可被 GLOB 收录。

## 集成建议

- 第三方只依赖所需模块（例如仅导航：`navigation.proto` + `common.proto`）。
- 所有响应包含 `automsgs.rpcs.common.Status`（`code` / `message`）。
