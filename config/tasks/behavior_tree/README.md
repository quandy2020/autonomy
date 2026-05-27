# 行为树 XML 与插件对照

`navigate_to_pose.xml` / `navigate_through_poses.xml` 中引用的**自定义节点**均在
`autonomy/tasks/behavior_tree/plugins/` 中实现，并在 `tasks.lua` 的
`TASKS_PLUGIN_LIB_NAMES` 中注册加载。

标准控制节点（`Sequence`、`ReactiveFallback`、`Inverter`、`ForceFailure`）由
BehaviorTree.CPP 内置提供，无需单独插件。

## navigate_to_pose.xml

| XML 节点 | 插件源文件 | 说明 |
|----------|------------|------|
| `RecoveryNode` | `plugins/control/recovery_node.cpp` | 失败重试 |
| `GoalReached` | `plugins/condition/goal_reached_condition.cpp` | 到点判定 |
| `InitialPoseReceived` | `plugins/condition/initial_pose_received_condition.cpp` | 初始位姿就绪 |
| `TransformAvailable` | `plugins/condition/transform_available_condition.cpp` | TF 可用 |
| `PipelineSequence` | `plugins/control/pipeline_sequence.cpp` | 流水线（子节点依次保持 RUNNING） |
| `ControllerSelector` | `plugins/action/controller_selector_node.cpp` | 选控制器（blackboard） |
| `PlannerSelector` | `plugins/action/planner_selector_node.cpp` | 选规划器 |
| `SmootherSelector` | `plugins/action/smoother_selector_node.cpp` | 选平滑器 |
| `RateController` | `plugins/decorator/rate_controller.cpp` | 限频重规划 |
| `ComputePathToPose` | `plugins/action/compute_path_to_pose_action.cpp` | 单目标规划 |
| `SmoothPath` | `plugins/action/smooth_path_action.cpp` | 路径平滑 |
| `IsPathValid` | `plugins/condition/is_path_valid_condition.cpp` | 路径碰撞检查 |
| `FollowPath` | `plugins/action/follow_path_action.cpp` | 路径跟踪 |
| `TimeExpired` | `plugins/condition/time_expired_condition.cpp` | 局部生存超时 |
| `RoundRobin` | `plugins/control/round_robin_node.cpp` | 轮询局部动作 |
| `DriveOnHeading` | `plugins/action/drive_on_heading_action.cpp` | 直走恢复 |
| `Spin` | `plugins/action/spin_action.cpp` | 原地旋转恢复 |
| `BackUp` | `plugins/action/back_up_action.cpp` | 后退恢复 |
| `ClearEntireCostmap` | `plugins/action/clear_costmap_service.cpp` | 清空代价地图 |
| `Wait` | `plugins/action/wait_action.cpp` | 延时 |

## navigate_through_poses.xml

| XML 节点 | 插件源文件 |
|----------|------------|
| `RecoveryNode` | `recovery_node.cpp` |
| `PipelineSequence` | `pipeline_sequence.cpp` |
| `ControllerSelector` / `PlannerSelector` / `SmootherSelector` | `*_selector_node.cpp` |
| `RateController` | `rate_controller.cpp` |
| `ComputePathThroughPoses` | `compute_path_through_poses_action.cpp` |
| `SmoothPath` | `smooth_path_action.cpp` |
| `IsPathValid` | `is_path_valid_condition.cpp` |
| `FollowPath` | `follow_path_action.cpp` |
| `ClearEntireCostmap` | `clear_costmap_service.cpp` |
| `Wait` | `wait_action.cpp` |

## Blackboard 键（由运行时注入）

| 键 | 来源 |
|----|------|
| `goal` / `goals` / `path` | 导航器 `OnGoalReceived` |
| `global_frame` / `robot_base_frame` | `PopulateBlackboardDefaults` |
| `goal_reached_tol` / `local_survival_timeout` | `tasks.lua` → TaskOptions |
| `selected_planner` / `selected_controller` / `selected_smoother` | Selector 节点 + 默认值 |
| `default_planner_id` / `default_controller_id` | `common.lua` |

## 修改行为树

1. 编辑本目录下 XML。
2. 若新增**自定义**节点：在 `plugins/` 增加 `.cpp` 并注册 `REGISTER_BEHAVIOR_TREE_NODE`，同步更新 `tasks.lua` 中 `TASKS_PLUGIN_LIB_NAMES`。
3. 仅调整流程时可复用现有节点，无需改 C++。

## 导航器入口

| XML 文件 | C++ 导航器 | Task API |
|----------|------------|----------|
| `navigate_to_pose.xml` | `navigator/navigate_to_pose.*` | `Task::StartNavigateToPose` |
| `navigate_through_poses.xml` | `navigator/navigate_through_poses.*` | `Task::StartNavigateThroughPoses` |

引擎：`behavior_tree/navigator/behavior_tree_navigation_engine.*`。
