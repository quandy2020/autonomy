(stopped-goal-checker)=
# 17. StoppedGoalChecker

> 归属 [§3.4 StoppedGoalChecker](../03_checkers.md#34-stoppedgoalchecker) · Autonomy ✅ 已实现
>
> **StoppedGoalChecker** 继承 SimpleGoalChecker：在 XY + 航向达标后，还要求**线速度与角速度**低于停止阈值，用于充电对接、电梯等须**停稳**才算到达的场景。

---

## 1. 背景

仅位姿进入容差时，机器人仍可能因惯性或控制器输出非零速度而微动。对接类任务要求「几何到位 **且** 静止」。Nav2 **StoppedGoalChecker** 在 Simple 判定之上叠加速度门限，常与 [§20 VelocitySmoother](../smoother/20_velocity_smoother_impl.md) 联用以更快满足停止条件。

---

## 2. 问题

**任务.** 判定位姿到达且底盘速度接近零。

**输入 / 输出.** `IsGoalReached(query, goal, velocity)` → `bool`；**必须使用** `velocity` 中的 $v_x,v_y,\omega_z$。

**在线形式.** 每周期：先位姿（含 stateful XY 锁定），再速度；全部满足才返回 true。

---

## 3. 位姿与速度模型

**位姿误差.** 与 [§15 SimpleGoalChecker](15_simple_goal_checker.md) 相同：$d_{xy}$、$\Delta\theta=\mathrm{AngleDiff}(\theta,\theta_g)$。

**速度量.**

$$
v_{trans} = \sqrt{v_x^2 + v_y^2}, \qquad \omega = \omega_z.
$$

- **$v_{trans}^{stop}$**：线速度停止阈值（默认 0.25 m/s）
- **$\omega_{rot}^{stop}$**：角速度停止阈值（默认 0.25 rad/s）

---

## 4. 数学问题定义

**位姿到达** $G_p$. SimpleGoalChecker 语义（XY + yaw，含 stateful）为真。

**速度停止** $G_v$.

$$
v_{trans} \leq v_{trans}^{stop}, \qquad |\omega| \leq \omega_{rot}^{stop}.
$$

**总到达条件.**

$$
\text{reached} \Leftrightarrow G_p \land G_v.
$$

时空联合控制器（TEB/NMPC）规划的速度剖面在终点应趋于零；若仅几何跟踪，须依赖 Smoother 或控制器减速区。

---

## 5. 判定算法

<div class="algorithm-box-diagram">

<div class="algorithm-box algorithm-box-phase-a">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 1</span>
    <span class="algorithm-box-title">StoppedGoalChecker::IsGoalReached</span>
  </div>
  <div class="algorithm-box-body" markdown="1">

1. **若** SimpleGoalChecker::IsGoalReached($q,g,v$) 为 false → 返回 false  
2. $v_{trans} \leftarrow \mathrm{hypot}(v_x, v_y)$  
3. **若** $v_{trans} \leq v_{trans}^{stop}$ ∧ $|\omega_z| \leq \omega_{rot}^{stop}$ → 返回 true  
4. **否则** 返回 false  

  </div>
</div>

</div>

`GetTolerances` 在 Simple 基础上写入线/角速度容差。精密对接可收紧至 $0.05$ m/s 量级；见 [§6.14 选型矩阵](../06_survey.md#614-工程选型矩阵) 中 StoppedGoalChecker 行。

---

## 6. 参考文献

1. Navigation2 Controller Server — Goal Checker plugins: [configuring-controller-server](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)
