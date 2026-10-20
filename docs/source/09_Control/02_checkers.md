# 2. 目标与进度检查器

Goal：是否到点。Progress：是否还在往前走（否 → `FailedToMakeProgress`）。代码在 `autonomy/control/checker/`。

```text
GoalChecker::IsGoalReached(pose, goal, vel) → bool
ProgressChecker::Check(pose) → bool   // true = 有进度
```

| 类 | 判据 | 文档 | ControllerServer |
|----|------|------|------------------|
| SimpleGoalChecker | XY + yaw | [01](checker/01_simple_goal_checker.md) | 默认加载 |
| PositionGoalChecker | 仅 XY | [02](checker/02_position_goal_checker.md) | 有类，未注册 |
| StoppedGoalChecker | 位姿 + 停稳 | [03](checker/03_stopped_goal_checker.md) | 有类，未注册 |
| SimpleProgressChecker | XY 位移 | [04](checker/04_simple_progress_checker.md) | 默认加载 |
| PoseProgressChecker | 位移或航向 | [05](checker/05_pose_progress_checker.md) | 有类，未注册 |

`controller_server.cpp` 里目前只 `make_shared` 前两者，并用 `checker_options` 写容差。

```lua
goal_checker = { xy_goal_tolerance = 0.25, yaw_goal_tolerance = 0.35, stateful = true }
progress_checker = { required_movement_radius = 0.5, movement_time_allowance = 10.0 }
```
