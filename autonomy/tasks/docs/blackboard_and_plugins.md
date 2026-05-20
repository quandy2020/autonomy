# Blackboard 与 BT 插件

## Blackboard 初始化

`SetupNavigateToPoseBlackboard()`（`common/bt_blackboard_setup.cpp`）在创建 `BehaviorTreeNavigator` 时调用，为 `navigate_to_pose.xml` 准备键值。

### 时间与超时

| 键 | 类型 | 来源 |
|----|------|------|
| `bt_loop_duration` | `milliseconds` | `TaskOptions.bt_loop_duration`（默认 10ms） |
| `server_timeout` | `milliseconds` | `TaskOptions.default_server_timeout` |
| `default_server_timeout` | `milliseconds` | 同上 |
| `wait_for_service_timeout` | `milliseconds` | `TaskOptions.wait_for_service_timeout` |

`BtActionServer::Run()` 使用成员 `bt_loop_duration_`（与 blackboard 同步设置）。

### 坐标系与生存

| 键 | 类型 | 说明 |
|----|------|------|
| `global_frame` | `string` | 默认 `map` |
| `robot_base_frame` | `string` | 默认 `base_link` |
| `local_survival_timeout` | `double` | 局部生存超时（秒），XML 中 `{local_survival_timeout}` |

### 导航数据

| 键 | 类型 | 说明 |
|----|------|------|
| `goal` | `PoseStamped` | 目标位姿，与 XML `{goal}` 对应 |
| `path` | `Path` | 规划结果，与 XML `{path}` 对应 |
| `selected_controller` | `string` | Selector 输出，默认 `FollowPath` |
| `selected_planner` | `string` | Selector 输出，默认 `navfn_planner` |
| `compute_path_error_code` / `compute_path_error_msg` | | ComputePathToPose 输出 |
| `follow_path_error_code` / `follow_path_error_msg` | | FollowPath 输出 |

### 状态

| 键 | 类型 | 说明 |
|----|------|------|
| `task_context` | `shared_ptr<TaskContext>` | 进程内服务入口 |
| `tf_buffer` | `shared_ptr<Buffer>` | TF |
| `odom_smoother` | `shared_ptr<OdomSmoother>` | 反馈估算 |
| `initial_pose_received` | `bool` | `InitialPoseReceived` 条件 |
| `number_recoveries` | `int` | 恢复计数 |

## TaskContext

插件通过 `taskContext()`（`BtStatefulActionNode`）或 blackboard `get("task_context")` 访问：

```cpp
struct TaskContext {
    shared_ptr<PlannerServer> planner;
    shared_ptr<ControllerServer> controller;
    shared_ptr<Costmap2DWrapper> global_costmap;
    shared_ptr<Costmap2DWrapper> local_costmap;
    shared_ptr<Buffer> tf;
    // selected_*_id, cancel_flag, frames, ...
};
```

`TaskScheduler::Initialize()` 中赋值；`local_costmap` 缺省时回退为 `global_costmap`。

## 插件实现类型

### 1. BtStatefulActionNode（推荐）

- **tick**：`onStart()` → 循环 `onRunning()` → 取消时 `onHalted()`
- **适用**：规划、跟踪、等待等长运行逻辑

| 节点 | 文件 |
|------|------|
| `ComputePathToPose` | `plugins/action/compute_path_to_pose_action.*` |
| `FollowPath` | `plugins/action/follow_path_action.*` |
| `Wait` | `plugins/action/wait_action.*` |

### 2. BtCostmapClearNode（同步）

继承 `BtStatefulActionNode`，在 `onStart()` 内完成并返回 SUCCESS。

| 节点 | 工具函数 |
|------|----------|
| `ClearEntireCostmap` | `ClearEntireCostmap()` → `resetLayers()` |
| `ClearCostmapAroundRobot` | `ClearCostmapAroundRobot()` |
| `ClearCostmapExceptRegion` | `ClearCostmapExceptRegion()` |

`service_name` 端口示例：`global_costmap/clear_costmap`、`local_costmap/clear_costmap`（按前缀选择 costmap）。

### 3. ConditionNode

| 节点 | 行为 |
|------|------|
| `GoalReached` | 比较当前位姿与 `{goal}` |
| `TransformAvailable` | 查询 `tf_buffer` |
| `InitialPoseReceived` | 读 `initial_pose_received` |
| `IsPathValid` | `IsPathValidOnCostmap(global_costmap, path)` |
| `TimeExpired` | 计时，用于局部生存窗口 |

### 4. BtActionNode / BtServiceNode（遗留桩）

- `ActionClient::AsyncSendGoal` 在独立线程中立即 `SUCCEEDED`
- 用于尚未迁移的 `Spin`、`BackUp`、`DriveOnHeading` 等
- **不适合**作为关键路径依赖，仅保证 Recovery 树结构可执行

## Planner ID 映射

XML 常用别名 `GridBased`，配置中实际插件 id 为 `navfn_planner`：

```cpp
// utils/planner_id_utils.hpp
ResolvePlannerId("GridBased", task_context->selected_planner_id)
// → "navfn_planner"
```

`PlannerSelector` 与 `ComputePathToPose` 均经此映射后调用 `PlannerServer::GetPlan`。

## 插件库注册

`config/tasks/tasks.lua` 中 `plugin_lib_names` 列表须与 CMake 安装的 BT 插件 `.so` 名称一致（前缀 `autonomy_behavior_tree_`）。  
`BehaviorTreeEngine` 通过 `registerFromPlugin` 加载。

## 添加新插件 checklist

1. 实现节点并 `BT_REGISTER_NODES` 注册 XML 标签名  
2. 将库名加入 `tasks.lua` → `plugin_lib_names`  
3. 若依赖 server，在 `onRunning` 中使用 `taskContext()`  
4. 若使用新 blackboard 键，在 `SetupNavigateToPoseBlackboard` 或 Navigator 中初始化  
5. 更新本文档与 [navigate_to_pose_execution.md](navigate_to_pose_execution.md)
