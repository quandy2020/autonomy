(stopped-goal-checker)=
# 3. StoppedGoalChecker

在 SimpleGoal（XY + yaw）之上，再要求 **线速度、角速度低于阈值** 才算到达。源码：`checker/stopped_goal_checker.*`。

在 SimpleGoal 位姿达标基础上，还要求线/角速度低于阈值，避免惯性滑移报 success。充电桩、对接等必须停稳的场景用；**Server 未默认注册**，需改 `controller_server.cpp` 或等价插件化。

## 判据

1. `SimpleGoalChecker::IsGoalReached` 为真；
2. $|v_x|,|v_y|\le\varepsilon_{\mathrm{trans}}$，$|\omega_z|\le\varepsilon_{\mathrm{rot}}$。

默认阈值约 0.25 m/s 与 0.25 rad/s（构造成员；完整参数加载 TODO 与 Nav2 对齐）。

## 何时用

- 充电桩、对接、电梯等 **必须停稳** 才报 success。
- 控制器惯性大，单纯位姿达标仍会滑一段。

**注意：** Server **未** 默认注册；需改 `controller_server.cpp` 或等价插件化。

## 常见坑

| 现象 | 处理 |
|------|------|
| 位姿到了任务不结束 | 速度仍高于阈值；略增 `trans_stopped_velocity` |
| 风阻/坡道微动 | 略增阈值或延长等待（上层逻辑） |
| 与 SimpleGoal 混用 conf | 只有 Stopped 生效时才会检速度 |

适用场景与 [01 SimpleGoal](01_simple_goal_checker.md) 互补：Simple 适合一般导航，Stopped 适合精密停靠。
