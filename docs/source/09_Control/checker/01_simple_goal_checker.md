(simple-goal-checker)=
# 1. SimpleGoalChecker

归属 [§3.2](../02_checkers.md)。 已实现：`checker/simple_goal_checker.*`。

### 定义

$d_{xy}^2=(x-x_g)^2+(y-y_g)^2$，$\Delta\theta=\mathrm{AngleDiff}(\theta,\theta_g)$。

1. 若 `check_xy_`：需 $d_{xy}^2\le\varepsilon_{xy}^2$；`stateful` 达标后锁定，不再检 XY 
2. 始终需 $|\Delta\theta|\le\varepsilon_\theta$

### 参数

| — | 默认 |
|--|------|
| `xy_goal_tolerance` | 0.25 m |
| `yaw_goal_tolerance` | 0.25 rad（Lua 常 0.35） |
| `stateful` | true |

新目标前 `Reset()`。

### 用法

```cpp
SimpleGoalChecker c; c.Initialize("goal_checker", nullptr);
c.SetTolerances(0.25, 0.35, true);
bool ok = c.IsGoalReached(query, goal, vel);
```
