(simple-goal-checker)=
# 15. SimpleGoalChecker

> 归属 [§6 检查器 · §6.2](../06_checkers.md#62-simplegoalchecker)
>
> `autonomy::control::checker::SimpleGoalChecker` 判定机器人是否到达目标：**XY 位置 + 航向**，支持 **stateful** 两阶段停车。  
> 公式见 [03_math.md §3.9](../03_math.md)。

| 维度 | 说明 |
|------|------|
| 源码 | `autonomy/control/checker/simple_goal_checker.*` |
| 接口 | `common::GoalChecker` |
| 状态 | ✅ 已实现；Lua 参数接线待完成 |

---

## 1. 架构

```
ControllerServer::IsGoalReached()
    │
    ▼
SimpleGoalChecker::IsGoalReached(query, goal, velocity)
    │
    ├─ Phase 1: XY 距离 ≤ ε_xy ?
    │     └─ stateful → check_xy_ = false
    │
    └─ Phase 2: |AngleDiff(θ, θ_g)| ≤ ε_θ ?
          └─ true → 到达
```

---

## 2. 数学原理（Step-by-Step）

### Step 1：XY 检测（`check_xy_ == true`）

$$
d_{xy}^2 = (x - x_g)^2 + (y - y_g)^2 \leq \varepsilon_{xy}^2
$$

不满足 → 返回 **false**。

### Step 2：Stateful 锁定

若 Step 1 通过且 `stateful_ == true`：设 `check_xy_ = false`，后续不再检 XY。

### Step 3：航向检测

$$
|\mathrm{AngleDiff}(\theta, \theta_g)| \leq \varepsilon_\theta
$$

满足 → **true**（到达）；否则 **false**。

### Step 4：Reset

`Reset()` 恢复 `check_xy_ = true`，用于新目标。

---

## 3. 默认参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `xy_goal_tolerance_` | 0.25 m | XY 容差 |
| `yaw_goal_tolerance_` | 0.25 rad | 航向容差 |
| `stateful_` | true | XY 锁定 |

`controller.lua` 顶层：

```lua
goal_checker = {
    xy_goal_tolerance = 0.25,
    yaw_goal_tolerance = 0.35,
    stateful = true,
},
```

> C++ `Initialize()` 当前硬编码默认值；临时用 `SetTolerances(xy, yaw, stateful)` 调参。

---

## 4. API

```cpp
auto checker = std::make_shared<checker::SimpleGoalChecker>();
checker->Initialize("goal_checker", nullptr);
checker->SetTolerances(0.25, 0.35, true);

bool reached = checker->IsGoalReached(query_pose, goal_pose, velocity);
checker->Reset();  // 新 FollowPath 前调用
```

---

## 5. 与其他 Checker 对比

| Checker | XY | Yaw | 速度 | Stateful |
|---------|----|----|------|----------|
| **SimpleGoalChecker** | ✅ | ✅ | ❌ | ✅ |
| PositionGoalChecker | ✅ | ❌ | ❌ | ✅ |
| StoppedGoalChecker | ✅ | ✅ | ✅ | ✅ |

---

## 6. 调参建议

| 场景 | xy | yaw |
|------|-----|-----|
| 室内通用 | 0.25 m | 0.35 rad |
| 精密停车 | 0.05 m | 0.1 rad |
| 只关心到位 | 用 PositionGoalChecker | — |

`xy_goal_tolerance` 应与 Navigator BT `GoalReached` 及 `AUTONOMY_COMMON.goal_reached_tolerance` 一致。

---

## 7. 源码索引

```69:88:autonomy/control/checker/simple_goal_checker.cpp
bool SimpleGoalChecker::IsGoalReached(...) {
  if (check_xy_) {
  // XY 检测 + stateful 锁定
  }
  double dyaw = AngleDiff(...);
  return std::abs(dyaw) <= yaw_goal_tolerance_;
}
```

---

## 8. 参考文献

1. Nav2 SimpleGoalChecker: [nav2_controller plugins](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)
