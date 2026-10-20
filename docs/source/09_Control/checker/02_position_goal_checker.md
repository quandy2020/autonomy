(position-goal-checker)=
# 2. PositionGoalChecker

只检查 **平面位置**，忽略最终航向。源码：`checker/position_goal_checker.*`。

只判平面距离是否在容差内，忽略目标 yaw；可选 stateful 锁定 XY。适合「开到附近即可、朝向由后续任务决定」；**Server 未默认加载**，仅改 conf 不会替换 SimpleGoal。

## 判据

到达 $\Leftrightarrow d_{xy}^2\le\varepsilon_{xy}^2$。可选 `stateful`：XY 达标后保持 true 直到 `Reset()`。

## 何时用

- 目标点只要求「开到附近」，朝向由后续动作或上层任务决定。
- 全向底盘或控制器自己会收敛朝向。

**注意：** `ControllerServer` **未** 默认加载此类；要在 Server 里注册并指定 `follow_goal_checker_id` 才会生效。

## 参数

`xy_goal_tolerance`（默认 0.25 m 量级），与 SimpleGoal 的 xy 语义相同。

## 常见坑

| 现象 | 说明 |
|------|------|
| 车斜着停也算到达 | 设计如此；要 yaw 用 SimpleGoal |
| 配置了无效 | Server 仍用 SimpleGoal |

```cpp
PositionGoalChecker c;
c.Initialize("goal_checker", nullptr);
c.SetTolerances(0.25, /*yaw unused*/ 0.0, true);
```
