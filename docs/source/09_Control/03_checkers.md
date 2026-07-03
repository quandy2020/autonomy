# 3. 目标与进度检查器

Checker 是 Control 模块的**判定子系统**，独立于局部控制器运行。Goal Checker 判定是否到达目标；Progress Checker 判定机器人是否在向目标前进（防卡住）。

> 各 Checker 实现专题见 `checker/15_*`–`19_*`（文件名前缀；与 `controller/15_mpc` 不同目录）。文档地图见 [00_guide §0.1](00_guide.md#01-阅读路径与文档地图)。

## 3.1 接口设计

### GoalChecker

```cpp
virtual bool IsGoalReached(
    const Pose& query_pose,
    const Pose& goal_pose,
    const Twist& velocity) = 0;

virtual bool GetTolerances(Pose& pose_tol, Twist& vel_tol) = 0;
virtual void Reset() = 0;
```

### ProgressChecker

```cpp
virtual bool Check(PoseStamped& current_pose) = 0;
virtual void Reset() = 0;
```

**语义约定**：

- GoalChecker：`true` = 已到达
- ProgressChecker：`true` = **有进度**（机器人移动足够）；`false` = 无进度 → 触发 `FailedToMakeProgress`

## 3.2 SimpleGoalChecker

实现细节、逐步判定与源码索引见 **[§15 SimpleGoalChecker](checker/15_simple_goal_checker.md)**。

**文件**：`checker/simple_goal_checker.*`

**默认参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `xy_goal_tolerance_` | 0.25 m | XY 位置容差 |
| `yaw_goal_tolerance_` | 0.25 rad | 航向容差 |
| `stateful_` | true | XY 达标后锁定 |

**判定流程**：

<div class="plan-arch-diagram">

  <div class="plan-arch-layer plan-arch-app">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">阶段 1</span>
      <span class="plan-arch-title">XY 位置检测</span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-body-block">
        <div class="nav-body-label">条件</div>
        <p><code>check_xy_ == true</code> 时检测</p>
        <p>$(x-x_g)^2 + (y-y_g)^2 \leq \varepsilon_{xy}^2$</p>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>XY 通过 + stateful → check_xy_ = false</span></div>

  <div class="plan-arch-layer plan-arch-server">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">阶段 2</span>
      <span class="plan-arch-title">航向检测</span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-body-block">
        <div class="nav-body-label">条件</div>
        <p>$|\mathrm{AngleDiff}(\theta, \theta_g)| \leq \varepsilon_\theta$</p>
      </div>
    </div>
  </div>

</div>

**Stateful 行为**：XY 达标后即使后续 XY 漂移超出容差，也不再重检 XY，只检航向。适用于"先到位、再转朝向"的两阶段停车策略。

**运行时调参**：

```cpp
checker->SetTolerances(0.25, 0.35, true);
checker->Reset();  // 重置 check_xy_ = true
```

## 3.3 PositionGoalChecker

实现细节见 **[§16 PositionGoalChecker](checker/16_position_goal_checker.md)**。

**文件**：`checker/position_goal_checker.*`

仅检测 XY 位置，**完全忽略航向**。

$$
(x - x_g)^2 + (y - y_g)^2 \leq \varepsilon_{xy}^2
$$

| 特性 | 说明 |
|------|------|
| `stateful_` | 一旦达标，`position_reached_ = true`，永久返回 true |
| 适用场景 | 只关心到达位置、不关心最终朝向的任务 |

## 3.4 StoppedGoalChecker

实现细节见 **[§17 StoppedGoalChecker](checker/17_stopped_goal_checker.md)**。

**文件**：`checker/stopped_goal_checker.*`

继承 `SimpleGoalChecker`，在 Simple 判定通过后追加**速度停止**条件：

$$
\sqrt{v_x^2 + v_y^2} \leq v_{trans}^{stop}, \quad
|\omega_z| \leq \omega_{rot}^{stop}
$$

| 参数 | 默认值 |
|------|--------|
| `trans_stopped_velocity_` | 0.25 m/s |
| `rot_stopped_velocity_` | 0.25 rad/s |

**适用场景**：需要机器人完全静止后才算到达（充电对接、精密操作）。

## 3.5 SimpleProgressChecker

实现细节见 **[§18 SimpleProgressChecker](checker/18_simple_progress_checker.md)**。

**文件**：`checker/simple_progress_checker.*`

维护基线位姿 `baseline_pose_`（Pose2D）：

```
Check(pose):
  if 首次调用 or IsRobotMovedEnough(pose):
      ResetBaselinePose(pose)
      return true    // 有进度
  else:
      return false   // 无进度 → 可能卡住
```

**移动判定**（仅 XY）：

$$
\operatorname{hypot}(x - x_b, y - y_b) > r
$$

| 参数 | 默认值 |
|------|--------|
| `radius_` | 0.5 m |

**与 Nav2 差异**：Nav2 的 `SimpleProgressChecker` 还包含 `movement_time_allowance`（时间窗口内必须移动）。Autonomy 当前 **未实现时间维度**，proto 中 `movement_time_allowance` 已预留。

## 3.6 PoseProgressChecker

实现细节见 **[§19 PoseProgressChecker](checker/19_pose_progress_checker.md)**。

**文件**：`checker/pose_progress_checker.*`

继承 `SimpleProgressChecker`，扩展判定条件：

$$
\operatorname{hypot}(x - x_b, y - y_b) > r
\;\;\lor\;\;
|\mathrm{NormalizeAngleDiff}(\theta - \theta_b)| > \Delta\theta_{req}
$$

| 参数 | 默认值 |
|------|--------|
| `radius_` | 0.5 m（继承） |
| `required_movement_angle_` | 0.5 rad |

**适用场景**：原地旋转也算"有进度"（例如先转向再前进的策略）。

## 3.7 Checker 对比

| Checker | 检测维度 | Stateful | 速度检测 | 专题 | 典型用途 |
|---------|----------|----------|----------|------|----------|
| SimpleGoalChecker | XY + Yaw | ✅ | ❌ | [§15](checker/15_simple_goal_checker.md) | 通用导航 |
| PositionGoalChecker | XY only | ✅ | ❌ | [§16](checker/16_position_goal_checker.md) | 只关心位置 |
| StoppedGoalChecker | XY + Yaw + Vel | ✅ | ✅ | [§17](checker/17_stopped_goal_checker.md) | 精确停车 |
| SimpleProgressChecker | XY 位移 | — | ❌ | [§18](checker/18_simple_progress_checker.md) | 防卡住 |
| PoseProgressChecker | XY + Yaw 变化 | — | ❌ | [§19](checker/19_pose_progress_checker.md) | 含旋转进度 |

## 3.8 配置与集成

**Lua 配置**（`controller.lua`）：

```lua
goal_checker = {
    xy_goal_tolerance = 0.25,
    yaw_goal_tolerance = 0.35,
    stateful = true,
},
progress_checker = {
    required_movement_radius = 0.5,
    movement_time_allowance = 10.0,
},
```

**Proto 定义**（`checker_options.proto`）：

```protobuf
message GoalCheckerOptions {
    bool stateful = 1;
    string plugin = 2;
    double xy_goal_tolerance = 3;
    double yaw_goal_tolerance = 4;
}
message ProgressCheckerOptions {
    string plugin = 1;
    double required_movement_radius = 2;
    double movement_time_allowance = 3;
}
```

> **当前限制**：C++ `Initialize()` 内使用硬编码默认值，标注 `TODO: Load parameters from configuration`。临时方案：构造后调用 `SetTolerances()` / `SetXYGoalTolerance()`。

## 3.9 在 FollowPath 中的调用时序

```
每个控制周期 (@ controller_frequency):
  1. progress_checker->Check(current_pose)
     └─ false → 累计无进度时间 → 超时 throw FailedToMakeProgress

  2. controller->ComputeVelocityCommands(..., goal_checker, ...)
     └─ goal_checker 传入控制器，用于控制器内部终点判定

  3. IsGoalReached()
     └─ goal_checker->IsGoalReached(pose, end_pose, velocity)
        └─ true → 退出循环
```

## 3.10 调参建议

| 场景 | xy_tolerance | yaw_tolerance | progress radius |
|------|-------------|---------------|-----------------|
| 室内通用 | 0.25 m | 0.35 rad | 0.5 m |
| 精密对接 | 0.05 m | 0.1 rad | 0.1 m |
| 开阔场地 | 0.5 m | 0.5 rad | 1.0 m |
| 原地转向任务 | 0.25 m | 0.35 rad | 0.5 m + PoseProgressChecker |

**注意**：`xy_goal_tolerance` 应与 Navigator BT 中 `GoalReached` 节点的容差保持一致（`AUTONOMY_COMMON.goal_reached_tolerance`）。

## 3.11 Goal Checker 判定（数学）

### SimpleGoalChecker

1. 若 `check_xy`：$d_{xy}^2=(x-x_g)^2+(y-y_g)^2$；若 $d_{xy}^2>\varepsilon_{xy}^2$ → **false**
2. XY 通过且 `stateful` → `check_xy=false`（锁定 XY）
3. $\Delta\theta=\mathrm{AngleDiff}(\theta,\theta_g)$；$|\Delta\theta|\leq\varepsilon_\theta$ → **true**

### PositionGoalChecker

仅检 $d_{xy}^2\leq\varepsilon_{xy}^2$；`stateful` 时达标后恒 **true**。不检航向。

### StoppedGoalChecker

先满足 SimpleGoalChecker，再检 $v_{trans}=\sqrt{v_x^2+v_y^2}\leq v_{trans}^{stop}$ 且 $|\omega_z|\leq\omega_{rot}^{stop}$。

## 3.12 Progress Checker 判定（数学）

### SimpleProgressChecker

$d=\mathrm{hypot}(x-x_b,y-y_b)$；若 $d>r$（`radius_`，默认 0.5 m）或首次调用 → 更新基线 $(x_b,y_b)$，返回 **true**；否则 **false**（无进度）。

### PoseProgressChecker

额外 $\Delta\theta=|\mathrm{NormalizeAngleDiff}(\theta-\theta_b)|$；若 $d>r$ **或** $\Delta\theta>\Delta\theta_{req}$（默认 0.5 rad）→ 重置基线，**true**。
