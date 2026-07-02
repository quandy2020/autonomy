# 2. 快速开始

### 2.1 三步启用

1. 编辑 `config/control/controller.lua`
2. 在 `config/autonomy.lua` 中设置 `control = AUTONOMY_CONTROLLER`
3. 启动后 `system::Autonomy` 自动构造 `ControllerServer`

### 2.2 最小配置

```lua
-- config/control/controller.lua
AUTONOMY_CONTROLLER = {
    controller_frequency = 20.0,
    failure_tolerance = 30.0,
    publish_zero_velocity = false,

    controller_plugins = {
        -- "id:ClassName" 格式，待插件实现后启用
        -- "graceful_controller:GracefulController",
    },

    goal_checker = {
        xy_goal_tolerance = 0.25,
        yaw_goal_tolerance = 0.35,
        stateful = true,
    },
    progress_checker = {
        required_movement_radius = 0.5,
        movement_time_allowance = 10.0,
    },

    -- 附加模式：共享 planner 全局 costmap
    costmap = { enabled = false },
}
```

```lua
-- config/autonomy.lua
AUTONOMY = {
    control = AUTONOMY_CONTROLLER,
}
```

### 2.3 C++ 直接使用

```cpp
#include "autonomy/control/controller_server.hpp"
#include "autonomy/control/control_options.hpp"

// 从 Lua 加载配置
auto dict = autonomy::common::LuaParameterDictionary::NonReferenceCounted(
    "config/control/controller.lua", autonomy::common::LoadLuaScript);
auto options = autonomy::control::LoadOptions(dict->GetDictionary("AUTONOMY_CONTROLLER").get());

auto server = std::make_shared<autonomy::control::ControllerServer>(options);
server->Start();

// 注入 planner 共享 costmap
server->SetSharedCostmap(planner_server->GetCostmapWrapper());

// 订阅里程计
server->UpdateOdometry(odom_msg);

// FollowPath（待 ComputeControl 完整实现）
// server->ComputeControl();
```

### 2.4 独立使用 Checker

Goal / Progress Checker 可在单元测试中独立实例化：

```cpp
#include "autonomy/control/checker/simple_goal_checker.hpp"

auto checker = std::make_shared<autonomy::control::checker::SimpleGoalChecker>();
checker->Initialize("goal_checker", nullptr);
checker->SetTolerances(0.25, 0.35, true);

commsgs::geometry_msgs::Pose query, goal;
commsgs::geometry_msgs::Twist vel;
bool reached = checker->IsGoalReached(query, goal, vel);
```

### 2.5 独立使用 VelocitySmoother

```cpp
#include "autonomy/control/utils/velocity_smoother.hpp"

autonomy::control::proto::VelocitySmootherOptions opts;
opts.set_smoothing_frequency(20.0);
opts.set_scale_velocities(true);
opts.add_max_velocity(0.5);
opts.add_max_velocity(0.0);
opts.add_max_velocity(2.5);
// ... 设置 accel/decel

autonomy::control::utils::VelocitySmoother smoother(opts);
// 输入命令后调用 smootherTimer() 获取平滑输出
```

### 2.6 当前限制

| 功能 | 状态 |
|------|------|
| FollowPath 完整循环 | 未实现 |
| 控制器插件加载 | 未实现 |
| `cmd_vel` 发布 | 未接线 |
| Checker 参数从 Lua 加载 | 未接线（硬编码默认值） |
| VelocitySmoother 定时器节点 | 未接线 |

完整 API 与配置说明见 [§4 使用指南](04_usage.md)。
