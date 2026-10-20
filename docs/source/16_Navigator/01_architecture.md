(navigator-architecture)=
# 1. 架构

**代码在哪：** `autonomy/task/navigation/`（导航 Navigator）、`autonomy/task/behavior_tree/`（引擎与插件）。文档章名仍叫 Navigator。
上手 [§0](00_guide.md)；BT 细节 [§2–§5](02_bt_algorithms.md)。

**本章讲什么：** 行为树如何坐在 TaskServer 和 planning/control 之间。

BT 在 autonomy/task/；TaskServer 编排。

### 边界

```text
TaskServer / Bridge / Action ──Goal──► BT Navigator ──tick──► Planner / Control / Map / TF
```

| 一侧 | 做什么 | 不做什么 |
|------|--------|----------|
| 上游 | 收 Goal、互斥、取消 | 不算路径、不发电流 |
| 下游 | 调 ComputePath、FollowPath、清图等 | 不实现具体规划/控制算法 |

与 Nav2 `bt_navigator` 对齐思路；配置帧名、容差与 `common.lua` / task conf 共用。

### 分层（实现）

```text
应用：TaskServer、Bridge GoalChannel、Action Client
→ 编排：BehaviorTreeNavigator、Muxer、navigate_to_pose / through_poses 树
→ 引擎：BtRunner / BtTaskApp、插件注册（~10 ms tick）
→ 插件：action / condition / control / decorator（.so 在 build/lib）
→ 子系统：PlannerServer、ControllerServer、costmap、TF
```

插件库名：`autonomy_behavior_tree_{category}_{stem}`，列表在 `navigator.pb.txt` 的 `plugin_lib_names` → [§5](05_bt_plugins.md)。

### 单点导航数据流（摘要）

```text
NavigateToPose → navigate_to_pose.xml
loop: TF? → ComputePath(限频) → Smooth → IsPathValid → FollowPath
      否则 → 局部生存（DriveOnHeading / Spin / BackUp）
→ Result / Recovery（清图、后退、旋转，次数见 XML）
```

多点：`navigate_through_poses.xml`，规划频率更高，**无**单点那套 LocalSurvival 前置。  
直驱调试：GetPlan → NotifyPath，不跑完整 BT（见 [17 · §6](../17_Tasks/06_execution_modes.md)）。

### 配置管线

```text
common.lua（帧/容差）→ navigator.pb.txt / 模块 conf → NavigatorOptions → TaskServer 内 BT 应用
```

| 常见字段 | 作用 |
|----------|------|
| `bt_loop_duration` | tick 周期（ms） |
| `local_survival_timeout` | TF 丢失后局部生存上限（秒） |
| `plugin_lib_names` | 要 dlopen 的 BT 插件 |
| `navigate_to_pose` / `through_poses` | BT XML 路径 |

XML 默认在 `autonomy/task/conf/behavior_tree/navigation/`。

### 状态与互斥

```text
Idle → Running → Completed | Failed | Canceled → Idle
```

`NavigatorMuxer`（或同等 muxer）：同一时刻通常只有一个导航类 Navigator 在跑新 Goal。  
Bridge 发导航前若 `GetActiveGoal` 非空，应先 Cancel 或等终态。

### 扩展

| 需求 | 做法 |
|------|------|
| 新 BT 节点 | 实现节点 → `BT_REGISTER_NODES` → CMake 出 `.so` → 写入 `plugin_lib_names` + Groot XML |
| 新顶层任务树 | 新 XML + 注册到对应 TaskApp / Navigator 配置 |
| 从 Bridge 发令 | [15 Bridge](../15_Bridge/rpcs/07_navigation_command.md) → Task GoalChannel |
