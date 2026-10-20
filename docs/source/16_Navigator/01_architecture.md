(navigator-architecture)=
# 1. 架构

上手 [§0](00_guide.md)；BT [§2–§5](02_bt_algorithms.md)。

### 1.1 位置与边界

```text
Task / Bridge ──Goal──► BtNavigator ──BT tick──► Planner / Control / Map
```

| 边界 | 说明 |
|------|------|
| 上游 | TaskServer、Bridge、Action Client |
| 下游 | planning / control / map / transform |
| 不负责 | 路径搜索、控制律、定位算法 |

约束：nav2 对齐 · 编排不实现算法 · `common.lua` 共享帧/容差 · 局部生存（TF 丢失不立即失败）。

### 1.2 实现状态

| 组件 | 状态 |
|------|------|
| Options / Interface / Muxer / BT XML | — |
| `BtEngine` / 插件 `.so` / 顶层 BtNavigator | — |
| 多进程联调 | — |
| `Autonomy` 进程内 BT | （由 TaskServer） |

### 1.3 分层

```text
应用：Task / Bridge / Action
→ 编排：BtNavigator + Muxer + 两个 BehaviorTreeNavigator
→ 引擎：BtEngine · BtActionServer · BtContext（~100 Hz tick）
→ 插件：52 节点（24 Action / 16 Condition / 3 Control / 7 Decorator）
→ 子系统：PlannerServer · ControllerServer · costmap · TF
```

插件命名：`autonomy_behavior_tree_{category}_{stem}` → [§5](05_bt_plugins.md)。

### 1.4 数据流

**BT 单点**（摘要）：

```text
NavigateToPose → RunWithGoal(navigate_to_pose.xml)
loop 10ms: TF? → ComputePath(5Hz) → Smooth → IsPathValid → FollowPath
: else → DriveOnHeading / Spin / BackUp（局部生存）
→ Result
```

**直驱**：GetPlan → NotifyPath（不 FollowPath）。 
**多点**：`navigate_through_poses.xml`，规划 10 Hz，无局部生存。

### 1.5 配置管线

```text
common.lua → navigator.lua → LoadOptions → NavigatorOptions → BtNavigator
```

| 字段 | 说明 |
|------|------|
| `bt_loop_duration` | tick ms |
| `local_survival_timeout` | 局部生存秒 |
| `plugin_lib_names` | 52 插件 |
| `navigate_to_pose` / `through_poses` | XML 路径 |

### 1.6 状态与互斥

```text
Idle → Running → Completed|Failed|Canceled → Idle
```

模式：`GoalReached?` → GlobalMode（TF OK）/ LocalSurvivalMode（TF 丢）→ 失败则 Recovery×8。 
`NavigatorMuxer`：同一时刻仅一个 Navigator 活跃。

### 1.7 集成与扩展

| 模块 | 集成点 |
|------|--------|
| planning | ComputePath* / IsPathValid / GetPlan |
| control | FollowPath / Spin / BackUp |
| map | clear_costmap / 碰撞检测 |
| Bridge | gRPC Goal |

新增插件：实现节点 → `BT_REGISTER_NODES` → CMake `.so` → `plugin_lib_names` + Groot XML。 
新增 Navigator：继承 `BehaviorTreeNavigator` → 注册 → 默认 XML。
