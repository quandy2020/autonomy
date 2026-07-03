(simple-goal-checker)=
# 15. SimpleGoalChecker

> 归属 [§3.2 SimpleGoalChecker](../03_checkers.md#32-simplegoalchecker) · Autonomy ✅ 已实现
>
> **SimpleGoalChecker** 在 FollowPath 每周期判定机器人是否到达目标：**XY 位置 + 航向**；`stateful` 模式下 XY 达标后锁定，仅继续检航向，对应「先到位、再对齐」两阶段停车。

---

## 1. 背景

局部控制器持续输出 `cmd_vel`，Navigator 需独立、可配置的**到达判定**，避免与控制器内部终点逻辑耦合。Nav2 将 Goal Checker 插件化；**SimpleGoalChecker** 为默认：同时约束平面位置与最终航向，并通过 **stateful** 避免到位后因控制抖动反复触发 XY 失败。

---

## 2. 问题

**任务.** 给定当前位姿 $q$、目标位姿 $g$ 与当前速度（本 Checker 不使用速度），判定 FollowPath 是否可结束。

**输入 / 输出.** `IsGoalReached(query_pose, goal_pose, velocity)` → `bool`；`true` 表示到达。另提供 `GetTolerances`、`Reset()`。

**在线形式.** 每个控制周期调用；新 FollowPath 或新目标前须 `Reset()` 恢复 `check_xy_`。

---

## 3. 位姿与误差模型

**位姿.** $q=(x,y,\theta)^\top$，$g=(x_g,y_g,\theta_g)^\top$，均在世界系 $SE(2)$。

**位置误差.**

$$
d_{xy}^2 = (x-x_g)^2 + (y-y_g)^2.
$$

**航向误差.**

$$
\Delta\theta = \mathrm{AngleDiff}(\theta,\,\theta_g), \quad \Delta\theta \in (-\pi,\,\pi].
$$

- **$\varepsilon_{xy}$**：XY 容差（默认 0.25 m）；**$\varepsilon_\theta$**：航向容差（默认 0.25 rad，Lua 常设 0.35 rad）。

---

## 4. 数学问题定义

**到达条件**（Nav2 SimpleGoalChecker 语义）分两阶段：

**阶段 A（XY）.** 当内部标志 `check_xy_` 为真时，要求

$$
d_{xy}^2 \leq \varepsilon_{xy}^2.
$$

若不满足 → **未到达**。若满足且 `stateful_` → 置 `check_xy_ = false`（**锁定 XY**，后续不再检位置）。

**阶段 B（航向）.** 始终要求

$$
|\Delta\theta| \leq \varepsilon_\theta.
$$

满足 → **到达**；否则 **未到达**。

Stateful 语义：XY 一度达标后，即使后续因控制误差 $d_{xy} > \varepsilon_{xy}$，仍只检航向直至 `Reset()`。

---

## 5. 判定算法

<div class="algorithm-box-diagram">

<div class="algorithm-box algorithm-box-phase-a">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 1</span>
    <span class="algorithm-box-title">SimpleGoalChecker::IsGoalReached</span>
  </div>
  <div class="algorithm-box-io" markdown="1">

**输入**：$q,\,g$（及未使用的 $v$）  
**输出**：`reached` ∈ {true, false}

  </div>
  <div class="algorithm-box-body" markdown="1">

1. **若** `check_xy_`：**若** $d_{xy}^2 > \varepsilon_{xy}^2$ → 返回 false  
2. **若** 步骤 1 通过且 `stateful_` → `check_xy_` ← false  
3. $\Delta\theta \leftarrow \mathrm{AngleDiff}(\theta,\theta_g)$  
4. **若** $|\Delta\theta| \leq \varepsilon_\theta$ → 返回 true；**否则** false  

  </div>
</div>

<div class="algorithm-box algorithm-box-phase-b algorithm-box-subroutine">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 2</span>
    <span class="algorithm-box-title">Reset</span>
  </div>
  <div class="algorithm-box-body" markdown="1">

`check_xy_` ← true（新目标 / 新 FollowPath 前调用）

  </div>
</div>

</div>

实现见 `autonomy/control/checker/simple_goal_checker.cpp`。Lua→Proto 接线见 [§3.13](../03_checkers.md#313-lua-proto-c-接线状态)；临时调参 `SetTolerances(xy, yaw, stateful)`。选型对照 [§3.7](../03_checkers.md#37-checker-对比)。

---

## 6. 参考文献

1. Navigation2 Controller Server — Goal Checker plugins: [configuring-controller-server](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)
