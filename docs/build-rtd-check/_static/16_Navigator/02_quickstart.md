# 2. 快速开始

### 2.1 三步启用

1. 编辑 `config/navigator/navigator.lua`（帧、容差、BT XML、插件列表）
2. 在 `config/autonomy.lua` 中 `include "navigator/navigator.lua"`
3. 启动 `system::Autonomy` 并调用 `NavigateToPose()`

### 2.2 最小配置

`config/common.lua` 与 `navigator.lua` 共享以下字段，**三处必须一致**：

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

### 2.3 直驱模式（当前默认）

`Autonomy::Configure()` 当前设置 `use_bt_navigation_ = false`：

```cpp
autonomy::system::RuntimeOptions runtime;
runtime.config_directory = "config";
runtime.use_bt_navigation = false;

auto autonomy = std::make_shared<autonomy::system::Autonomy>(options);
autonomy->Configure(runtime);

commsgs::geometry_msgs::PoseStamped goal;
bool ok = autonomy->NavigateToPose(goal, [](){return false;}, [](){return true;}, 300.0);
```

直驱路径：`GetRobotPose` → `TransformPoseToGlobalFrame` → `GetPlan` → `NotifyPath`。

### 2.4 BT 模式（目标设计）

```bash
export AUTONOMY_BT_PLUGIN_PATH=/path/to/install/lib
bazel run //autonomy/system/tools:nav_test -- --config_directory=config --use_bt=true
```

### 2.5 切换导航模式

| 模式 | 配置 | 行为 |
|------|------|------|
| 直驱规划 | `use_bt_navigation = false` | 单次 `GetPlan`，不执行 FollowPath |
| BT 单点 | `use_bt_navigation = true` | `navigate_to_pose.xml` 完整流水线 |
| BT 多点 | `NavigateThroughPoses()` | `navigate_through_poses.xml` |
