(position-goal-checker)=
# 16. PositionGoalChecker

> 归属 [§3 检查器 · §3.3](../03_checkers.md#33-positiongoalchecker)
>
> `autonomy::control::checker::PositionGoalChecker` 仅判定 **XY 位置**是否到达，**完全忽略航向**。  
> 对比见 [§15 SimpleGoalChecker](15_simple_goal_checker.md)。

| 维度 | 说明 |
|------|------|
| 源码 | `autonomy/control/checker/position_goal_checker.*` |
| 接口 | `common::GoalChecker` |
| 状态 | ✅ 已实现 |

---

## 1. 架构

```
IsGoalReached(query, goal, velocity)
    │
    ├─ stateful && position_reached_ ? → true（永久锁定）
    │
    └─ d_xy² ≤ ε_xy² ?
          ├─ yes + stateful → position_reached_ = true
          └─ return position_reached
```

与 `SimpleGoalChecker` 差异：**无航向阶段**，`velocity` 参数未使用。

---

## 2. 数学原理（Step-by-Step）

### Step 1：Stateful 短路

若 `stateful_ && position_reached_`，直接返回 **true**。

### Step 2：XY 距离

$$
d_{xy}^2 = (x - x_g)^2 + (y - y_g)^2 \leq \varepsilon_{xy}^2
$$

### Step 3：锁定状态

若 Step 2 通过且 `stateful_`：设 `position_reached_ = true`。

### Step 4：Reset

`Reset()` 将 `position_reached_ = false`。

---

## 3. 默认参数

| 参数 | 默认 |
|------|------|
| `xy_goal_tolerance_` | 0.25 m |
| `stateful_` | true |

```cpp
checker->SetXYGoalTolerance(0.25);
checker->Reset();
```

---

## 4. 适用场景

| 场景 | 说明 |
|------|------|
| 物料搬运 | 只要求到达点位，朝向无关 |
| 巡检拍照 | 到达后另由其他逻辑调整朝向 |
| 物流停靠 | 后续人工或机械臂接管 |

**不适用**：需要精确最终朝向的停车（应使用 `SimpleGoalChecker` 或 `StoppedGoalChecker`）。

---

## 5. 与 SimpleGoalChecker 对比

| | PositionGoalChecker | SimpleGoalChecker |
|---|---------------------|-------------------|
| XY | ✅ | ✅ |
| Yaw | ❌ | ✅ |
| Stateful 语义 | 位置永久锁定 | XY 锁定后检 Yaw |
| GetTolerances 朝向 | 零四元数 | yaw 容差 |

---

## 6. 源码索引

```cpp
// autonomy/control/checker/position_goal_checker.cpp:46-66
bool PositionGoalChecker::IsGoalReached(...) {
  if (stateful_ && position_reached_) return true;
  bool position_reached = (dx*dx + dy*dy <= xy_goal_tolerance_sq_);
  if (stateful_ && position_reached) position_reached_ = true;
  return position_reached;
}
```

---

## 7. 参考文献

1. Nav2 PositionGoalChecker: [controller server plugins](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)
