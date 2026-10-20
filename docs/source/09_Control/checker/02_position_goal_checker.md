(position-goal-checker)=
# 2. PositionGoalChecker

归属 [§3.3](../02_checkers.md)。 仅检 XY，忽略航向。

### 定义

到达 $\Leftrightarrow d_{xy}^2\le\varepsilon_{xy}^2$。可选 stateful（到位后保持 true 直至 `Reset`）。

### 参数 / 用法

`xy_goal_tolerance`（默认 0.25 m）。适用：朝向无关的点到点。

```cpp
PositionGoalChecker c; c.Initialize(...); c.SetTolerances(0.25, /*unused*/0, true);
```
