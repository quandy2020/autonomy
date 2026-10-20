# 2. 目标与进度检查器

FollowPath 每周期用两类 checker：**Goal** 判是否到达终点；**Progress** 判是否在向目标移动。失败时 Progress 抛 `FailedToMakeProgress`，上层常取消导航。 FollowPath 每周期并行问两件事：Goal 是否到达、Progress 是否仍在向目标推进。`ControllerServer` **默认只注册** SimpleGoalChecker 与 SimpleProgressChecker；对接停稳或原地大角转向需换 StoppedGoal / PoseProgress 并改 Server 注册。

```text
GoalChecker::IsGoalReached(pose, goal, vel) → bool
ProgressChecker::Check(pose) → bool   // true = 仍有进度
```

实现目录：`autonomy/control/checker/`。

## 仓库里有哪些、Server 默认挂谁

| 类 | 判据 | 文档 | ControllerServer |
|----|------|------|------------------|
| SimpleGoalChecker | XY + yaw（可 stateful 锁 XY） | [01](checker/01_simple_goal_checker.md) | **默认加载** |
| PositionGoalChecker | 仅 XY | [02](checker/02_position_goal_checker.md) | 有类，未注册 |
| StoppedGoalChecker | SimpleGoal + 线/角速度低于阈值 | [03](checker/03_stopped_goal_checker.md) | 有类，未注册 |
| SimpleProgressChecker | 相对基准 XY 位移 | [04](checker/04_simple_progress_checker.md) | **默认加载** |
| PoseProgressChecker | XY 或航向变化 | [05](checker/05_pose_progress_checker.md) | 有类，未注册 |

`controller_server.cpp` 的 `LoadPlugins()` 只对前两者 `make_shared`，容差来自 `checker_options`（proto）。要用 StoppedGoal（ docking 必须停稳）或 PoseProgress（原地大角度转向），需改 Server 注册或自行 fork 接线。

## 默认配置（Lua / pb）

```lua
goal_checker = { xy_goal_tolerance = 0.25, yaw_goal_tolerance = 0.35, stateful = true }
progress_checker = { required_movement_radius = 0.5, movement_time_allowance = 10.0 }
```

C++ 默认：未配 proto 时 xy 0.10、yaw 0.20、`path_length_tolerance` 0.35（SimpleGoal 内部字段）。

## 怎么选

| 需求 | 建议 |
|------|------|
| 一般点到点 | 保持 SimpleGoal + SimpleProgress |
| 不关心最终朝向 | PositionGoal（需接线） |
| 充电、精密停靠要停稳 | StoppedGoal（需接线） |
| 接近目标前大角度原地转 | PoseProgress 替代 SimpleProgress（需接线） |

## 常见坑

| 现象 | 先查 |
|------|------|
| 到 xy 还在转 | yaw 容差过大；或 stateful 已锁 xy 仍在调角 |
| 明明到了还不停 | 路径终点与 Goal 不一致；checker 用的是 FollowPath 末 pose |
| 几秒就 FailedToMakeProgress | 膨胀把路堵死；`required_movement_radius` 过大；`movement_time_allowance` 过短 |
| 改了 StoppedGoal conf 无效 | Server 未加载该类 |

新目标下发前应对 checker 调 `Reset()`（Server 在 `ComputeControl` 里已调）。
