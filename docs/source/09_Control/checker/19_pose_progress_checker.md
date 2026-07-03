(pose-progress-checker)=
# 19. PoseProgressChecker

> 归属 [§3.6 PoseProgressChecker](../03_checkers.md#36-poseprogresschecker) · Autonomy ✅ 已实现
>
> **PoseProgressChecker** 继承 SimpleProgressChecker：除 XY 位移外，**航向变化** $\Delta\theta$ 超过阈值也视为有效进度，避免「先原地转向再前进」策略被误判为卡住。

---

## 1. 背景

Graceful / RPP 等控制器常采用**先对齐航向、再前进**；此阶段 XY 位移很小，SimpleProgressChecker 会持续返回 false。PoseProgressChecker 将转角纳入进度，与 Nav2 `nav2_controller` 中同名插件语义一致。

---

## 2. 问题

**任务.** 判定自基线以来是否发生足够 **平移或旋转**。

**输入 / 输出.** `Check(current_pose)` → `bool`；`true` = 有进度。

**在线形式.** 与 Simple 相同基线更新逻辑；扩展 `IsRobotMovedEnough` 为析取条件。

---

## 3. 位姿与位移模型

**XY 位移**（同 §18）.

$$
d_{xy} = \mathrm{hypot}(x - x_b,\; y - y_b).
$$

**航向变化.**

$$
\Delta\theta = \left|\mathrm{NormalizeAngleDiff}(\theta - \theta_b)\right|.
$$

- **$r$**：位移半径（默认 0.5 m，继承 Simple）
- **$\Delta\theta_{req}$**：`required_movement_angle`（默认 0.5 rad）

---

## 4. 数学问题定义

**有进度条件.**

$$
d_{xy} > r \;\;\lor\;\; \Delta\theta > \Delta\theta_{req}.
$$

满足 → 重置基线，返回 **true**；否则 **false**（无进度）。

---

## 5. 判定算法

<div class="algorithm-box-diagram">

<div class="algorithm-box algorithm-box-phase-a">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 1</span>
    <span class="algorithm-box-title">PoseProgressChecker::IsRobotMovedEnough</span>
  </div>
  <div class="algorithm-box-body" markdown="1">

1. $d_{xy} \leftarrow \mathrm{hypot}(x-x_b, y-y_b)$  
2. $\Delta\theta \leftarrow |\mathrm{NormalizeAngleDiff}(\theta - \theta_b)|$  
3. **若** $d_{xy} > r$ **或** $\Delta\theta > \Delta\theta_{req}$ → 返回 true  
4. **否则** 返回 false  

`Check()` 外层逻辑同 [§18 算法 1](18_simple_progress_checker.md#5-判定算法)：首次或 IsRobotMovedEnough 为真时 ResetBaselinePose。

  </div>
</div>

</div>

实现见 `pose_progress_checker.cpp`。与 Simple 选型见 [§3.7](../03_checkers.md#37-checker-对比)。

---

## 6. 参考文献

1. Navigation2 Controller Server — Progress Checker plugins: [configuring-controller-server](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)
