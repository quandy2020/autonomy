(simple-goal-checker)=
# 1. SimpleGoalChecker

`ControllerServer` **默认** 使用的 Goal checker。源码：`checker/simple_goal_checker.*`。

平面距离 + 航向差同时满足才算到达；`stateful=true` 时 XY 达标后会锁定，之后主要检 yaw。**ControllerServer 默认 Goal checker**。不关心最终朝向时改 PositionGoalChecker（需自行注册）。

## 判据

平面距离 $d_{xy}^2=(x-x_g)^2+(y-y_g)^2$，航向差 $\Delta\theta=\mathrm{AngleDiff}(\theta,\theta_g)$。

1. 若 `check_xy_` 为真：需 $d_{xy}^2\le\varepsilon_{xy}^2$；`stateful=true` 时 XY 一旦达标会 **锁定**，之后只检 yaw。
2. 始终要求 $|\Delta\theta|\le\varepsilon_\theta$。

另有 `path_length_tolerance`（Server 默认可设 0.35 m）用于沿路径长度相关逻辑，见源码 `IsGoalXYReached`。

## 参数（常用）

| 参数 | 典型值 |
|------|--------|
| `xy_goal_tolerance` | 0.25 m（Lua）；C++ 构造默认 0.25 |
| `yaw_goal_tolerance` | 0.35 rad（Lua 常见） |
| `stateful` | true |

新目标或新 FollowPath 前应 `Reset()`（Server 已在 `ComputeControl` 调用）。

## 何时用

默认室内导航、需要最终朝向对齐的场景。若 **不关心 yaw**，改用 PositionGoalChecker（需自行注册到 Server）。

## 常见坑

| 现象 | 处理 |
|------|------|
| 到点附近来回扭 | 略增 xy 容差或先 stateful 锁 xy |
| 永远不到点 | yaw 容差过严；或 goal 与 path 末 pose 不一致 |
| 改 conf 不生效 | 确认走 `checker_options.goal_checker` 且 Server 已重载 |

```cpp
SimpleGoalChecker c;
c.Initialize("goal_checker", nullptr);
c.SetTolerances(0.25, 0.35, true, 0.35);
bool ok = c.IsGoalReached(query, goal, vel);
```
