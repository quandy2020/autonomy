# 5. BT 插件节点

目录：`autonomy/navigator/behavior_tree/plugins/`。命名：`autonomy_behavior_tree_{category}_{stem}`。

加载：`navigator.lua` → `plugin_lib_names`（52）+ `AUTONOMY_BT_PLUGIN_PATH`。 
Groot：`config/navigator/behavior_tree/autonomy_tree_nodes.xml`。

### 5.1 数量

| 类别 | 数量 |
|------|------|
| action | 24 |
| condition | 16 |
| control | 3 |
| decorator | 7 |

### 5.2 Action（24）

| 节点 | 目标 | 主要端口 |
|------|------|----------|
| `ComputePathToPose` | Planner | `goal`, `path`, `planner_id` |
| `ComputePathThroughPoses` | Planner | `goals`, `path` |
| `SmoothPath` | Smoother | `unsmoothed_path`, `smoothed_path` |
| `TruncatePath` / `TruncatePathLocal` | 本地 | 裁剪距离 |
| `GetPoseFromPath` | 本地 | `path`, `index` |
| `FollowPath` | Controller | `path`, `controller_id`, `goal_checker_id` |
| `Spin` / `BackUp` / `DriveOnHeading` | Controller | 距离/速度/时限 |
| `Wait` | 定时 | `wait_duration` |
| `*Cancel` | 取消对应 Action | — |
| `Planner/Controller/Smoother/GoalChecker/ProgressChecker Selector` | Topic 选型 | `selected_*` |
| `ClearEntireCostmap` | clear 服务 | local/global |
| `RemovePassedGoals` / `RemoveInCollisionGoals` | 航点 | — |

### 5.3 Condition（16）

| 节点 | 判定 |
|------|------|
| `GoalReached` | $d_{xy}\le\varepsilon$ |
| `TransformAvailable` | `canTransform` |
| `InitialPoseReceived` | 初位姿标志 |
| `IsPathValid` / `IsStuck` / `IsStopped` | 路径 / 卡住 / 停止 |
| `GoalUpdated` / `GloballyUpdatedGoal` | 目标更新 |
| `TimeExpired` / `PathExpiringTimer` / `DistanceTraveled` / `ArePosesNear` | 时空 |
| 恢复决策类 | 是否进入恢复 |
| 电源类（可选） | 电量相关 |

### 5.4 Control / Decorator

| Control | 语义 |
|---------|------|
| `PipelineSequence` | 流水线推进 |
| `RecoveryNode` | 失败恢复重试 |
| `RoundRobin` | 轮转 |

| Decorator | 用途 |
|-----------|------|
| `RateController` | 限频 |
| `SingleTrigger` / `GoalUpdatedController` 等 | 触发与目标更新 |

### 5.5 子系统映射

| 子系统 | 代表节点 |
|--------|----------|
| Planning | ComputePath* · IsPathValid |
| Control | FollowPath · Spin · BackUp · DriveOnHeading |
| Map | ClearEntireCostmap |
| TF | TransformAvailable |

### 5.6 多点树 `navigate_through_poses.xml`

```text
Recovery → Pipeline：Selector → ComputePathThroughPoses(10Hz)
→ Smooth → IsPathValid → FollowPath
+ RemovePassedGoals / RemoveInCollisionGoals
```

与单点差异：无 LocalSurvival、无 GoalReached 前置、规划 10 Hz。

### 5.7 扩展

实现节点 → 注册宏 → CMake `.so` → `plugin_lib_names` + Groot XML。
