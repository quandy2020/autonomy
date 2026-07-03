(simple-progress-checker)=
# 18. SimpleProgressChecker

> 归属 [§3.5 SimpleProgressChecker](../03_checkers.md#35-simpleprogresschecker) · Autonomy ✅ 已实现（`movement_time_allowance` 待接）
>
> **SimpleProgressChecker** 检测机器人在 FollowPath 中是否产生**足够 XY 位移**；无位移则返回 false，累计后 Navigator 抛出 `FailedToMakeProgress`，触发 Recovery。

---

## 1. 背景

局部控制器可能因障碍、错误 lookahead 或定位异常而**原地振荡或卡住**，Goal Checker 长期 false 却难以区分「仍在努力」与「已失效」。Progress Checker 维护**基线位姿**，若位移低于阈值则判为无进度，与 Goal Checker **并行**运行（语义相反：`true` = 有进度）。

---

## 2. 问题

**任务.** 每周期判断自上次基线以来是否移动足够远。

**输入 / 输出.** `Check(current_pose)` → `bool`；`true` = **有进度**，`false` = **无进度**。

**在线形式.** 首次调用或检测到足够位移时更新基线 $(x_b,y_b)$；Autonomy 当前**未实现** Nav2 的 `movement_time_allowance` 时间窗口（proto 已预留）。

---

## 3. 位姿与位移模型

**当前位姿.** $p=(x,y,\theta)^\top$；基线 $p_b=(x_b,y_b,\theta_b)^\top$（仅 XY 参与 Simple 判定）。

**平面位移.**

$$
d_{xy} = \mathrm{hypot}(x - x_b,\; y - y_b).
$$

- **$r$**：`required_movement_radius`（默认 0.5 m）。

---

## 4. 数学问题定义

**有进度条件.**

$$
d_{xy} > r \quad \Longrightarrow \quad \text{更新基线，返回 true}.
$$

**无进度.** $d_{xy} \leq r$ 且非首次调用 → 返回 false。

Nav2 完整语义还包括：在 $T_{allow}$（`movement_time_allowance`）内必须至少一次满足 $d_{xy}>r$，否则失败。Autonomy C++ **待实现**该计时逻辑。

---

## 5. 判定算法

<div class="algorithm-box-diagram">

<div class="algorithm-box algorithm-box-phase-a">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 1</span>
    <span class="algorithm-box-title">SimpleProgressChecker::Check</span>
  </div>
  <div class="algorithm-box-body" markdown="1">

1. **若** 首次调用（`baseline_pose_set_` 为 false）→ ResetBaselinePose($p$) → 返回 true  
2. $d_{xy} \leftarrow \mathrm{hypot}(x-x_b, y-y_b)$  
3. **若** $d_{xy} > r$ → ResetBaselinePose($p$) → 返回 true  
4. **否则** 返回 false（无进度）  

  </div>
</div>

</div>

原地大角度旋转而 XY 几乎不变时，Simple 会误判无进度 → 改用 [§19 PoseProgressChecker](19_pose_progress_checker.md)。FollowPath 集成时序见 [§3.9](../03_checkers.md#39-在-followpath-中的调用时序)。

---

## 6. 参考文献

1. Navigation2 Controller Server — Progress Checker plugins: [configuring-controller-server](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)
