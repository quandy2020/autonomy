(navigator-usage)=
# 4. 使用指南

### 4.1 配置

**入口文件**：`config/navigator/navigator.lua` → `config/autonomy.lua`

```lua
AUTONOMY = {
    navigator = include("navigator/navigator.lua").navigator,
}
```

**关键字段**：

| 字段 | 说明 | 默认值 |
|------|------|--------|
| `global_frame` | 全局规划/TF 父帧 | `map`（来自 `common.lua`） |
| `robot_base_frame` | 机器人基座帧 | `base_link` |
| `bt_loop_duration` | BT tick 周期 (ms) | `10` |
| `default_server_timeout` | 子 Action 超时 (ms) | `20000` |
| `local_survival_timeout` | 局部生存最长等待 (s) | `120.0` |
| `goal_reached_tolerance` | BT GoalReached 容差 (m) | `0.25` |
| `plugin_lib_names` | BT 插件 `.so` 列表 | 52 个 |
| `navigate_to_pose.behavior_tree_file` | 单点 BT XML | `navigate_to_pose.xml` |
| `navigate_through_poses.behavior_tree_file` | 多点 BT XML | `navigate_through_poses.xml` |

C++ 加载：

```cpp
auto options = autonomy::navigator::CreateOptions("config", "navigator/navigator.lua");
```

或通过 `AutonomyOptions` 嵌入：

```cpp
// system::Autonomy 构造后
autonomy->Configure(runtime);  // 内部调用 CreateOptions + ApplyRuntimeToNavigatorOptions
```

### 4.2 Autonomy API

`system::Autonomy` 是进程内导航的顶层入口。

| API | 用途 |
|-----|------|
| `Configure(RuntimeOptions)` | 加载 navigator 配置，应用运行时覆盖 |
| `UseBehaviorTreeNavigation()` | 查询是否 BT 模式 |
| `NavigateToPose(goal, cancel, keep_alive, timeout)` | 单点导航 |
| `NavigateThroughPoses(goals, cancel, keep_alive, timeout)` | 多点顺序导航 |
| `ReplanToGoal(goal)` | 触发重规划（直驱/BT 均可用） |
| `GetLastPath()` | 获取最近一次规划路径 |
| `RequestCancelNavigation()` | 取消当前导航 |
| `TransformPoseToGlobalFrame(pose)` | 坐标变换到 `global_frame` |
| `TickControl()` | 单步控制 tick（仿真/离线） |

**RuntimeOptions 覆盖**：

```cpp
autonomy::system::RuntimeOptions runtime;
runtime.config_directory = "config";
runtime.use_bt_navigation = true;   // 目标：启用 BT（当前 Configure 强制 false）
runtime.global_frame = "map";
runtime.planner_id = "navfn_planner";
runtime.controller_id = "FollowPath";
runtime.goal_checker_id = "goal_checker";
runtime.goal_tolerance = 0.25;
autonomy->Configure(runtime);
```

**当前默认行为**：`Autonomy::Configure()` 设置 `use_bt_navigation_ = false`，`NavigateToPose()` 走 `NavigateDirectToPose()`（仅规划，不 FollowPath）。BT 完整流水线待 `BtNavigator` 恢复后启用。

### 4.3 NavigatorInterface

所有 Navigator 实现统一生命周期接口（`common/interface.hpp`）：

```cpp
class NavigatorInterface {
public:
    enum class NavigatorState {
        kIdle, kRunning, kCompleted, kFailed, kCanceled, kShutdown,
    };
    virtual NavigatorState GetState() const = 0;
    virtual bool Cancel() = 0;
    virtual void Shutdown() = 0;
};
```

| 状态 | 含义 |
|------|------|
| `kIdle` | 无活跃导航 |
| `kRunning` | BT tick 循环或 Action 执行中 |
| `kCompleted` | 目标到达（SUCCESS） |
| `kFailed` | 恢复耗尽或超时 |
| `kCanceled` | 用户取消 |
| `kShutdown` | 节点关闭 |

`BehaviorTreeNavigator<ActionT>` 模板基类实现 `NavigatorInterface` 语义，通过 `BtActionServer` 驱动 BT。

### 4.4 Action 接口

#### 4.4.1 顶层导航 Action

| Action | Goal | Feedback | Result |
|--------|------|----------|--------|
| `NavigateToPoseAction` | `pose`, `behavior_tree` | `current_pose`, `distance_remaining`, `number_of_recoveries`, … | `error_code`, `error_msg` |
| `NavigateThroughPosesAction` | `poses[]`, `behavior_tree` | 同上 + `number_of_poses_remaining` | 同上 |

Goal 中 `behavior_tree` 为空时使用 `navigator.lua` 默认 XML。

#### 4.4.2 BT 调用的子 Action

| Action | BT 节点 | 服务端 |
|--------|---------|--------|
| `ComputePathToPoseAction` | `ComputePathToPose` | `PlannerServer` |
| `ComputePathThroughPosesAction` | `ComputePathThroughPoses` | `PlannerServer` |
| `SmoothPathAction` | `SmoothPath` | Smoother |
| `FollowPathAction` | `FollowPath` | `ControllerServer` |
| `SpinAction` / `BackUpAction` / `DriveOnHeadingAction` | 恢复原语 | `ControllerServer` |

#### 4.4.3 Service

| Service | BT 节点 | 说明 |
|---------|---------|------|
| `IsPathValid` | `IsPathValid` | 路径碰撞检测 |
| `ClearEntireCostmap` | `ClearEntireCostmap` | 清局部/全局 costmap |

### 4.5 系统集成

```cpp
// system::Autonomy 启动链
map_server_   = MapServer(options.map_options());
planner_      = PlannerServer(options.planner_options());
controller_   = ControllerServer(options.controller_options());
tf_buffer_    = transform::Buffer::Instance();

// Configure 时加载 navigator 配置（BtNavigator 待恢复）
navigator_options_ = navigator::CreateOptions(config_dir, "navigator/navigator.lua");
```

**直驱模式数据流**（当前默认）：

```
NavigateToPose()
  → TransformPoseToGlobalFrame(goal)
  → GetRobotPose(start)
  → PlannerServer::GetPlan(start, goal)
  → NotifyPath(path)
```

**BT 模式数据流**（目标设计）：

```
NavigateToPose()
  → BtNavigator
  → NavigateToPoseNavigator::StartWithGoal()
  → BtActionServer::RunWithGoal()
  → tick(navigate_to_pose.xml)
  → ComputePath / FollowPath / Recovery …
```

### 4.6 通信接口

| 类型 | 名称 | 说明 |
|------|------|------|
| 节点 | `navigator` | autolink 节点（`kNavigatorNodeName`） |
| Action | `navigate_to_pose` | 单点导航 |
| Action | `navigate_through_poses` | 多点巡航 |
| Topic | `controller_selector` | 运行时切换控制器 |
| Topic | `planner_selector` | 运行时切换规划器 |
| Topic | `smoother_selector` | 运行时切换平滑器 |
| Service | `is_path_valid` | 路径有效性（经 BT 调用） |
| Service | `*/clear_costmap` | 恢复清图 |

进程内模式（`BtActionServer`）Goal/Feedback/Result 在内存中传递，无需 autolink 序列化；分布式部署时通过 autolink Action Client/Server 桥接。

### 4.7 BT 插件环境

52 个 BT 插件由 `navigator.lua` 的 `plugin_lib_names` 注册，CMake 编译为独立 `.so`。

**环境变量**：

```bash
export AUTONOMY_BT_PLUGIN_PATH=/path/to/install/lib
```

当 `plugin_lib_path` 为空时，`BtEngine` 从 `AUTONOMY_BT_PLUGIN_PATH` 搜索插件库。

**离线测试**：

```bash
export AUTONOMY_BT_PLUGIN_PATH=/workspace/autonomy/build/lib
autonomy_nav_test \
  --configuration_directory=config \
  --goal_x=5 --goal_y=5 \
  --use_bt=true
```

**插件加载顺序**：

```
navigator.lua (plugin_lib_names)
      │
      ▼
BtEngine::LoadPlugins(path, names)
      │
      ▼
BT_REGISTER_NODES(factory)  ← 各 .so 导出
      │
      ▼
BehaviorTreeFactory 注册 52 个节点类型
```

### 4.8 自定义行为树

1. 复制 `config/navigator/behavior_tree/navigate_to_pose.xml` 为新文件
2. 按需修改节点组合（参考 [07_navigate_to_pose.md](07_navigate_to_pose.md)）
3. 在 `navigator.lua` 中更新 `behavior_tree_file`，或通过 Goal 的 `behavior_tree` 字段运行时指定
4. 新增节点时同步更新 `autonomy_tree_nodes.xml`（Groot 模型）和 `plugin_lib_names`

**最小改动示例**（提高规划频率）：

```xml
<RateController hz="10.0">
  <ComputePathToPose goal="{goal}" path="{path}" planner_id="{selected_planner}"/>
</RateController>
```

**注意**：自定义 XML 引用的 `{blackboard_key}` 须在 `PopulateBlackboardDefaults()` 或 `OnGoalReceived()` 中注入，否则 tick 时端口解析失败。

### 4.9 导航模式选型

| 场景 | 模式 | 配置 | 理由 |
|------|------|------|------|
| 规划算法调试 | 直驱 | `use_bt_navigation = false` | 跳过 BT，快速验证 GetPlan |
| 标准室内导航 | BT 单点 | `navigate_to_pose.xml` | 完整流水线 + 恢复 |
| 定位不稳定 | BT 单点 + 增大超时 | `local_survival_timeout = 180` | 局部生存等待重定位 |
| 仓库巡检 | BT 多点 | `navigate_through_poses.xml` | 10 Hz 重规划 |
| 动态拥挤环境 | BT + 高频规划 | `RateController hz=10` | 快速响应障碍变化 |
| 窄通道 | BT + Theta* | `planner_selector → theta_star_planner` | 任意角路径 |
| 远程调度 | BT + Bridge gRPC | 待实现 | 云端下发目标 |

### 4.10 故障排查

| 错误码 | 名称 | 常见原因 | 处理 |
|--------|------|----------|------|
| 9001 | `NAV_TO_POSE_NOT_INITIALIZED` | BtNavigator 未 Configure | 检查 `Autonomy::Configure()` |
| 9002 | `NAV_TO_POSE_TIMEOUT` | 超过 `timeout_sec` | 增大超时或检查卡住原因 |
| 9003 | `NAV_TO_POSE_CANCELED` | 用户调用 Cancel | 正常行为 |
| 9004 | `NAV_TO_POSE_PREEMPTED` | 新 Goal 抢占 | 正常行为 |
| 9005 | `NAV_TO_POSE_FAILED_TO_LOAD_BEHAVIOR_TREE` | XML 路径错误或插件缺失 | 检查 BT 文件与 `AUTONOMY_BT_PLUGIN_PATH` |
| 9006 | `NAV_TO_POSE_TF_ERROR` | map→base_link 变换不可用 | 检查 localization / TF 发布 |
| 9007 | `NAV_TO_POSE_INVALID_GOAL` | Goal 帧错误或位姿无效 | 检查 `frame_id` 与坐标 |
| 9008 | `NAV_TO_POSE_NO_VALID_PATH` | 规划失败 | 检查 costmap、起终点 |
| 9009 | `NAV_TO_POSE_PLANNER_FAILED` | ComputePathToPose FAILURE | 见 Planning [§4.8](../08_Planning/04_usage.md#48-故障排查) |
| 9010 | `NAV_TO_POSE_CONTROLLER_FAILED` | FollowPath FAILURE | 见 Control [§4.8](../09_Control/04_usage.md) |
| 9011 | `NAV_TO_POSE_SMOOTHER_FAILED` | SmoothPath 超时/碰撞 | 增大 `max_smoothing_duration` |
| 9012 | `NAV_TO_POSE_PATH_INVALID` | IsPathValid FAILURE | 清图或重规划 |
| 9013 | `NAV_TO_POSE_GOAL_CHECKER_FAILED` | 航向无法对齐 | 增大 `yaw_goal_tolerance` |

**运行时日志检查清单**：

| 现象 | 检查项 |
|------|--------|
| 直驱无路径 | `GetPlan failed` → costmap / 起终点 |
| `no robot pose` | Controller 里程计 / TF 是否注入 |
| BT 插件加载失败 | `plugin_lib_names` 与 `.so` 文件名一致 |
| 局部生存超时 | `local_survival_timeout` 120 s 内 TF 未恢复 |
| 容差不一致 | `common.lua`、`controller.lua`、`navigator.lua` 三处 `goal_reached_tolerance` |
| 多 Navigator 冲突 | `NavigatorMuxer` 拒绝并发 Goal |

**nav_test 常见错误**（`autonomy/system/tools/README.md`）：

| 日志 | 原因 | 处理 |
|------|------|------|
| `NavigateDirectToPose: no robot pose` | 无里程计 | nav_test 已注入初始 odom |
| BT 模式立即失败 | 插件路径未设置 | 设置 `AUTONOMY_BT_PLUGIN_PATH` |

### 4.11 性能建议

| 建议 | 说明 |
|------|------|
| `bt_loop_duration = 10` ms | 100 Hz tick，平衡响应与 CPU |
| 规划 5 Hz | 静态环境可降至 2 Hz |
| 直驱调试 | 跳过 BT 与 FollowPath，仅验证规划 |
| 容差 0.25 m | 室内通用；精密场景 0.10 m |
| 局部生存 120 s | 定位慢恢复可增至 180 s |

### 4.12 common.lua 参数契约

`config/common.lua` 是 **planner / controller / navigator 三模块的单一事实来源**，以下字段必须保持一致：

```lua
AUTONOMY_COMMON = {
    global_frame = "map",
    robot_base_frame = "base_link",
    default_planner_id = "navfn_planner",
    default_controller_id = "FollowPath",
    default_goal_checker_id = "goal_checker",
    default_smoother_id = "simple_smoother",
    goal_reached_tolerance = 0.25,
}
```

| 字段 | navigator 用途 | controller 用途 | 不一致后果 |
|------|---------------|-----------------|-----------|
| `global_frame` | TF 检验、黑板注入 | costmap 全局帧 | 规划/跟踪坐标系错位 |
| `robot_base_frame` | TransformAvailable | 机器人 pose 查询 | TF 链断裂 |
| `default_planner_id` | PlannerSelector 默认 | — | 规划器 id 不匹配 |
| `default_controller_id` | ControllerSelector 默认 | FollowPath 默认 | 控制器 id 不匹配 |
| `default_goal_checker_id` | FollowPath 端口 | GoalChecker 插件 | 到达判定插件错误 |
| `goal_reached_tolerance` | BT `goal_reached_tol` | `xy_goal_tolerance` | BT SUCCESS 与 Controller 不同步 |

**引用方式**（`navigator.lua`）：

```lua
if AUTONOMY_COMMON == nil then
    include "common.lua"
end

navigator = {
    global_frame = AUTONOMY_COMMON.global_frame,
    goal_reached_tolerance = AUTONOMY_COMMON.goal_reached_tolerance,
    -- ...
}
```

**controller.lua 对应**：

```lua
xy_goal_tolerance = AUTONOMY_COMMON.goal_reached_tolerance,
```

修改容差时 **只改 `common.lua` 一处**，三模块自动同步。

### 4.13 相关文档

- [快速开始](02_quickstart.md)
- [模块架构](05_architecture.md)
- [行为树引擎](06_bt_engine.md)
- [单点导航 BT](07_navigate_to_pose.md)
