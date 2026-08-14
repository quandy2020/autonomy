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
| Task | 并入 **`SystemService`** |
| 落地强度 | **方案 1**：领域文件 + 统一异步约定；不保留 deprecated 空壳 |
| 传感器文件名 | 保留 **`sensor.proto`** |
| 回充 | `charging` → **`docking`**，语义见充电 A 设计 |

## 3. 文件地图

| 文件 | package | Service | 职责 |
|---|---|---|---|
| `common.proto` | `automsgs.rpcs.common` | — | `Status`, `Code` |
| `navigation.proto` | `automsgs.rpcs.navigation` | `NavigationService` | 点到点导航 |
| `mapping.proto` | `automsgs.rpcs.mapping` | `MappingService` | 建图 + 地图 CRUD + 当前图 |
| `localization.proto` | `automsgs.rpcs.localization` | `LocalizationService` | 定位状态、地图系位姿、重定位 |
| `docking.proto` | `automsgs.rpcs.docking` | `DockingService` | 回充对接 |
| `follow.proto` | `automsgs.rpcs.follow` | `FollowService` | 人体/目标跟随 |
| `sensor.proto` | `automsgs.rpcs.sensor` | `SensorService` | 传感器列举与快照 |
| `system.proto` | `automsgs.rpcs.system` | `SystemService` | 心跳、系统/硬件、通用 Task |

**删除并迁出：** `status.proto`、`charging.proto`、`world.proto`、`vehicle.proto`、`task.proto`。

## 4. 统一约定

1. **命名：** 文件/包 `lower_snake`；Service/RPC/消息 PascalCase；字段 snake_case；枚举值 `TYPE_PREFIX_*`，零值 `_UNSPECIFIED`。  
2. **msgs：** 位姿用 `geometry_msgs.Pose` / `PoseWithCovariance`；地图用 `map_msgs.OccupancyGrid` / `MapMetaData`；传感用 `sensor_msgs.*`；时间戳用 `std_msgs.Header`；电量优先 `sensor_msgs.BatteryState`。  
3. **长任务：** `Start*` / 域惯用名（`GoTo`/`Dock`）立即返回 `goal_id`；`Cancel(goal_id)`（空=当前）；`GetStatus`；单一活动目标，忙则拒绝（不抢占）。  
4. **响应：** 皆含 `common.Status`。  
5. **Code 区间：** 保留现有；新增 follow `900–999`、system `1000–1099`、mapping 建图过程 `1100–1199`（地图存储仍 `600–699`）。

## 5. 命令面摘要

### 5.1 NavigationService

- `GoTo(target_pose, goal_id?)` → `goal_id`  
- `Cancel(goal_id?)`  
- `GetStatus` → `NavigationState`, `goal_id`, `remaining_distance`

### 5.2 MappingService

- 建图：`StartMapping` / `StopMapping` / `GetMappingStatus`（异步 goal）  
- CRUD：`ListMaps`, `GetMap`, `GetMapMetadata`, `SaveMap`, `DeleteMap`, `SetCurrentMap`  
- 载荷：`OccupancyGrid`, `MapMetaData`；`MapSummary` 留在 mapping 包内

### 5.3 LocalizationService

- `GetPose(map_frame_id?)` → `Header` + `Pose`（可选 covariance）  
- `GetStatus` → `LocState`（LOCALIZED/LOST/INITIALIZING）, `map_id`  
- `SetInitialPose(pose, map_id?)` — 重定位

### 5.4 DockingService

见充电 A 设计：`Dock` / `Undock` / `Cancel` / `GetStatus`；状态含 DOCKING/UNDOCKING/CHARGING/FULL/FAILED/CANCELLED 等；Code 500–507。

### 5.5 FollowService

- `StartFollow(target_id?, goal_id?)` — 空 target=跟最近人体  
- `Cancel` / `GetStatus` — `FollowState`: IDLE/FOLLOWING/LOST_TARGET/FAILED/CANCELLED

### 5.6 SensorService

- `ListSensors` / `GetImage` / `GetLaserScan` / `GetPointCloud` / `GetOccupancyGrid`  
- 在现有 API 上统一 RPC 动词前缀；`GetGrid` 更名为 `GetOccupancyGrid`

### 5.7 SystemService

- `Ping` — 轻量存活  
- `Heartbeat` — 可选携带机器人侧时间戳/序列号  
- `GetInfo` — 型号、序列号、软/硬件版本、主机名等  
- `GetStatus` — 总运行态（IDLE/BUSY/ERROR/ESTOP…）+ 电量摘要  
- `RunTask` / `CancelTask` / `GetTaskStatus` — 原 TaskService（`task_id`/`run_id`/`task_args`）

## 6. 迁移与实现顺序

1. `common.proto`（自 status 改名）+ Code 新区间  
2. `docking.proto`（落实充电 A）  
3. `mapping.proto` + `localization.proto`（自 world 拆分 + 建图 RPC）  
4. `navigation.proto` / `sensor.proto` 微调  
5. 新建 `follow.proto`；充实 `system.proto`  
6. 删除旧文件；重写 `rpcs/README.md`

**兼容：** 本轮接受 breaking（改名/删文件/改 package）。不提供 deprecated 转发文件。

## 7. 验收清单

- [ ] 仅保留 §3 所列 8 个 proto（无 world/vehicle/task/charging/status）  
- [ ] 覆盖导航、建图、地图 CRUD、定位/重定位、回充、跟随、传感器、心跳/系统/硬件/Task  
- [ ] 长任务统一异步模型  
- [ ] 载荷优先 `automsgs/msgs`  
- [ ] 命名符合 Google Style 与机器人领域用语  
- [ ] README 与 Code 区间文档同步  

## 8. 后续模块实现

总览批准后，按 §6 顺序开实施计划；每模块可有独立小 plan，但以本 spec 为权威命令面。  
Docking 细节以 `2026-08-14-charging-rpc-nav-align-design.md` 为准（文件/服务名改为 docking）。
