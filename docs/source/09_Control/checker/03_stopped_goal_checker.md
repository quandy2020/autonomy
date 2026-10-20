(stopped-goal-checker)=
# 3. StoppedGoalChecker

归属 [§3.4](../02_checkers.md)。 SimpleGoal + 速度停稳。

### 定义

先满足 SimpleGoal（XY + yaw），再要求 $|v_x|,|v_y|,|\omega|\le\varepsilon_v$。

### 参数

沿用 Simple 容差；另设线/角速度阈值（Nav2 `rot_stopped_velocity` / `trans_stopped_velocity` 类）。

适用：充电、精密停靠
