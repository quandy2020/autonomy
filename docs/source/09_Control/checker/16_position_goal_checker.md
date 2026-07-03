(position-goal-checker)=
# 16. PositionGoalChecker

> 归属 [§3.3 PositionGoalChecker](../03_checkers.md#33-positiongoalchecker) · Autonomy ✅ 已实现
>
> **PositionGoalChecker** 仅判定 **XY 位置**是否进入容差，**完全忽略航向**；`stateful` 时位置一度达标后永久返回 true，直至 `Reset()`。

---

## 1. 背景

部分任务只要求到达空间点位（物料停靠、巡检到点），最终朝向由后续动作或人工处理。**PositionGoalChecker** 避免控制器为对齐航向而长时间原地旋转，缩短 FollowPath 周期。

---

## 2. 问题

**任务.** 判定平面位置是否到达目标，不约束 $\theta$。

**输入 / 输出.** `IsGoalReached(query, goal, velocity)` → `bool`；`velocity` 未使用。

**在线形式.** 每控制周期调用；`Reset()` 清除 `position_reached_` 锁定。

---

## 3. 位姿与误差模型

**位姿.** 仅使用 $(x,y)$；目标 $(x_g,y_g)$。

**位置误差.**

$$
d_{xy}^2 = (x-x_g)^2 + (y-y_g)^2.
$$

- **$\varepsilon_{xy}$**：XY 容差（默认 0.25 m）。

---

## 4. 数学问题定义

**Stateful 短路.** 若 `stateful_` 且 `position_reached_` 已为真 → **到达**（不再计算距离）。

**位置条件.**

$$
d_{xy}^2 \leq \varepsilon_{xy}^2.
$$

若满足且 `stateful_` → `position_reached_` ← true。

**到达**当且仅当上述判定为真（或已处于锁定状态）。不检验 $|\Delta\theta|$。

---

## 5. 判定算法

<div class="algorithm-box-diagram">

<div class="algorithm-box algorithm-box-phase-a">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 1</span>
    <span class="algorithm-box-title">PositionGoalChecker::IsGoalReached</span>
  </div>
  <div class="algorithm-box-body" markdown="1">

1. **若** `stateful_` ∧ `position_reached_` → 返回 true  
2. **若** $d_{xy}^2 \leq \varepsilon_{xy}^2$：  
   - **若** `stateful_` → `position_reached_` ← true  
   - 返回 true  
3. **否则** 返回 false  

  </div>
</div>

</div>

`GetTolerances` 对朝向返回零四元数（无 yaw 容差）。需最终朝向时用 [§15 SimpleGoalChecker](15_simple_goal_checker.md) 或 [§17 StoppedGoalChecker](17_stopped_goal_checker.md)。

---

## 6. 参考文献

1. Navigation2 Controller Server — Goal Checker plugins: [configuring-controller-server](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)
