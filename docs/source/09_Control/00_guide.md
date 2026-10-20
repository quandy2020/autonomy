(control-guide)=
# 0. Control 运动控制指南

`autonomy/control` 做局部跟踪：吃全局 Path，吐 `cmd_vel`。架构见 [01](01_architecture.md)，插件总表见 [04](04_controller_algorithms.md)。

当前插件：Graceful、MPPI、RPP、NMPC、NeuPAN、SANDO、Policy。Goal / Progress Checker 与 VelocitySmoother 见 [02](02_checkers.md)、[03](03_velocity_smoother.md)。

```
Path → ControllerServer → cmd_vel
         ↑ Checker ·（可选）Smoother
```

差速模型 $\dot x=v\cos\theta,\ \dot y=v\sin\theta,\ \dot\theta=\omega$。跟踪可写成对位姿误差与控制量的二次型：

$$
\min_u\int_0^T\big(e_p^\top Q e_p+w_\theta e_\theta^2+u^\top R u\big)\,dt
$$

室内一般用 RPP / Graceful；要躲动态障碍用 MPPI 或 NeuPAN；要硬约束或走廊用 NMPC / SANDO；自己训策略接 Policy。

```lua
AUTONOMY_CONTROLLER = {
    controller_frequency = 20.0,
    goal_checker = { xy_goal_tolerance = 0.25, yaw_goal_tolerance = 0.35, stateful = true },
    progress_checker = { required_movement_radius = 0.5, movement_time_allowance = 10.0 },
}
```

```cpp
auto server = std::make_shared<ControllerServer>(LoadOptions(...));
server->Start();
server->SetSharedCostmap(planner->GetCostmapWrapper());
```

排错：到不了点先查容差和 GoalChecker；`FailedToMakeProgress` 看障碍、膨胀和 Progress 时限；抖看加速度界；没 `cmd_vel` 查插件名和发布接线。
