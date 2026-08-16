# automsgs RPCs 总览设计 — 第三方机器人业务 API

日期：2026-08-14  
状态：已批准  
范围：`automsgs/proto/rpcs/*`（重组与完备）  
关联：`docs/superpowers/specs/2026-08-14-charging-rpc-nav-align-design.md`（对接细节并入 `docking`）

## 1. 背景与目标

为第三方集成提供完备、少文件、易维护的 unary RPC 面，覆盖：导航、建图、地图 CRUD、定位/重定位、自动回充、人体跟随、传感器快照、系统心跳/状态/硬件信息及通用任务。

**目标：**

- 文件少且按机器人领域划分  
- Google Protocol Buffers 风格命名  
- 优先复用 `automsgs/proto/msgs`  
- 长任务统一异步模型（对齐 `NavigationService`）  
- 第三方仅依赖所需模块即可完成业务编排  

**非目标（本总览不展开实现细节）：** streaming/topic 订阅、语音 TTS/ASR（可后续加 `speech.proto`）、地图编辑几何图元高级 API。

## 2. 已确认决策

| 项 | 选择 |
|---|---|
| 交付方式 | 先总览设计，再按模块改 proto |
| 文件粒度 | 方案 A（约 8 文件），命名偏专业 |
| 公共文件 | `status.proto` → **`common.proto`** |
| Task | **删除**通用 Task；急停进 `SystemService` |
| 落地强度 | **方案 1**：领域文件 + 统一异步约定；不保留 deprecated 空壳 |
| 传感器文件名 | 保留 **`sensor.proto`** |
| 回充 | **`charge`**（`ChargeService`：`Return`/`Leave`），见 `2026-08-15-charge-rpc-rename-design.md` |

## 3. 文件地图

| 文件 | package | Service | 职责 |
|---|---|---|---|
| `common.proto` | `automsgs.rpcs.common` | — | `Status`, `Code` |
| `navigation.proto` | `automsgs.rpcs.navigation` | `NavigationService` | 点到点导航 |
| `mapping.proto` | `automsgs.rpcs.mapping` | `MapService` | StartMapping/FinishMapping + 地图存储 |
| `localization.proto` | `automsgs.rpcs.localization` | `LocalizationService` | 定位状态、地图系位姿、重定位 |
| `charge.proto` | `automsgs.rpcs.charge` | `ChargeService` | 自动回充 |
| `follow.proto` | `automsgs.rpcs.follow` | `FollowService` | 人体/目标跟随 |
| `sensor.proto` | `automsgs.rpcs.sensor` | `SensorService` | 传感器列举与快照 |
| `system.proto` | `automsgs.rpcs.system` | `SystemService` | 心跳、急停/解除、CancelAll、ActiveGoal、Capabilities |
| `teleop.proto` | `automsgs.rpcs.teleop` | `TeleopService` | 遥控 Drive（bidi） |
| `exploration.proto` | `automsgs.rpcs.exploration` | `ExplorationService` | 自主探索建图 |

**删除并迁出：** `status.proto`、`charging.proto`、`world.proto`、`vehicle.proto`、`task.proto`。

## 4. 统一约定

1. **命名：** 文件/包 `lower_snake`；Service/RPC/消息 PascalCase；字段 snake_case；枚举值 `TYPE_PREFIX_*`，零值 `_UNKNOWN`。  
2. **msgs：** 位姿用 `geometry_msgs.Pose` / `PoseWithCovariance`；地图用 `map_msgs.OccupancyGrid` / `MapMetaData`；传感用 `sensor_msgs.*`；时间戳用 `std_msgs.Header`；电量优先 `sensor_msgs.BatteryState`。  
3. **长任务：** `Start*` / 域惯用名（`Navigate`/`Return`）立即返回或开流；`Cancel(goal_id)`（空=当前）；`GetStatus`；单一活动目标，忙则拒绝（不抢占）。  
4. **响应：** 皆含 `common.Status`。  
5. **Code：** 统一 `status_msgs.StatusCode`（RPC `Status.code`）；成功 `OK=0`。

## 5. 命令面摘要

### 5.1 NavigationService

- `GoTo(target_pose, goal_id?)` → `goal_id`  
- `Cancel(goal_id?)`  
- `GetStatus` → `NavigationState`, `goal_id`, `remaining_distance`

### 5.2 MapService

- 会话：`StartMapping` / `FinishMapping` / `CancelMapping` / `GetMappingStatus`（`MappingStatus`）  
- 存储：`ListMaps` / `GetMap` / `GetMapMetadata` / `SaveMap` / `DeleteMap` / `SetCurrentMap`  
- 优先 `map_identifier`；`map_name` 为查找别名；忙 → `CODE_MAPPING_BUSY`（1100）

### 5.3 LocalizationService

- `GetPose(map_frame_id?)` → `PoseWithCovariance` + optional `confidence`  
- `GetStatus` → `LocalizationStatus`（`LocalizationState`、`map_id`、`confidence`）  
- `SetInitialPose(PoseWithCovariance, map_id?)` — 重定位；INITIALIZING 时忙 → `CODE_LOCALIZATION_BUSY`（803）

### 5.4 ChargeService

- `Return` / `Leave`（server stream `ChargeResponse`）/ `Pause` / `Resume` / `Cancel` / `GetStatus`  
- `ChargeState`：含 `PAUSED`；GetStatus 复用 `ChargeResponse`；Cancel 直接回 `Status`  
- Code 500–507（`CODE_CHARGING_*`）

### 5.5 FollowService

- `Follow`（server stream `FollowResponse`）/ `Pause` / `Resume` / `Cancel` / `GetStatus`  
- 无 `FollowPhase`；`FollowOptions.desired_distance_m`；`LOST_TARGET`/`PAUSED` 不关流  
- Code 900–903

### 5.6 SensorService

- `ListSensors` / `GetSample` — 统一快照（`oneof` 载荷）  
- `GetParameters` / `SetParameters` / `SaveParameters` / `LoadParameters` — 键值配置与机器人侧持久化  
- `Record`（stream）/ `CancelRecord` / `GetRecordStatus` — 录制/导出/上传  
- 细节见 `2026-08-14-sensor-rpc-design.md`；栅格走 mapping，不设按类型 Get*

### 5.7 SystemService

- `Heartbeat` / `GetInfo` / `GetStatus` / `GetHealth`  
- `EmergencyStop` / `ClearEmergencyStop`  
- `CancelAllGoals` / `GetActiveGoal` / `GetCapabilities`  
- `SystemHealth` 对齐 `autonomy/system/monitor`（Hazard / MRM / 主机与通道）

### 5.8 TeleopService

- `Velocity`（bidi）+ `VelocityOptions`；相对：`DriveOnHeading` / `BackUp` / `Spin` + `RelativeOptions`  
- `Pause` / `Resume`（相对，`PAUSED` 不关流）/ `Cancel` / `GetStatus`  
- Code 1200–1205

### 5.9 ExplorationService

- `Explore`（server stream）/ `Pause` / `Resume` / `Cancel` / `GetStatus`  
- `SetArea` / `SaveMap`（会话内存图；CRUD 仍 `MapService`）  
- Code 1300–1305；细节见 `2026-08-15-exploration-rpc-design.md`

## 6. 迁移与实现顺序

1. `common.proto`（自 status 改名）+ Code 新区间  
2. `charge.proto`（自动回充 `ChargeService`）  
3. `mapping.proto` + `localization.proto`（自 world 拆分 + 建图 RPC）  
4. `navigation.proto` / `sensor.proto` 微调  
5. 新建 `follow.proto`；充实 `system.proto`  
6. 删除旧文件；重写 `rpcs/README.md`

**兼容：** 本轮接受 breaking（改名/删文件/改 package）。不提供 deprecated 转发文件。

## 7. 验收清单

- [ ] 仅保留 §3 所列 8 个 proto（无 world/vehicle/task/charging/status）  
- [ ] 覆盖导航、建图、地图 CRUD、定位/重定位、回充、跟随、传感器、心跳/系统/硬件/急停  
- [ ] 长任务统一异步模型  
- [ ] 载荷优先 `automsgs/msgs`  
- [ ] 命名符合 Google Style 与机器人领域用语  
- [ ] README 与 Code 区间文档同步  

## 8. 后续模块实现

总览批准后，按 §6 顺序开实施计划；每模块可有独立小 plan，但以本 spec 为权威命令面。  
Charge 细节以 `2026-08-15-charge-rpc-rename-design.md` 为准。
