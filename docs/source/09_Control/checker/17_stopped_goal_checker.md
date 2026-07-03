(stopped-goal-checker)=
# 17. StoppedGoalChecker

> 归属 [§3 检查器 · §3.4](../03_checkers.md#34-stoppedgoalchecker)
>
> `autonomy::control::checker::StoppedGoalChecker` 继承 `SimpleGoalChecker`，在 XY + 航向达标后还要求**线速度与角速度接近零**。

| 维度 | 说明 |
|------|------|
| 源码 | `autonomy/control/checker/stopped_goal_checker.*` |
| 基类 | `SimpleGoalChecker` |
| 状态 | ✅ 已实现 |

---

## 1. 架构

```
IsGoalReached(query, goal, velocity)
    │
    ├─ SimpleGoalChecker::IsGoalReached()  → false ? 返回 false
    │
    ├─ |v_trans| = hypot(vx, vy) ≤ v_trans_stop ?
    │
    └─ |ω_z| ≤ ω_rot_stop ?
          └─ 均满足 → true
```

---

## 2. 数学原理（Step-by-Step）

### Step 1：位姿判定（继承 Simple）

与 [§15 SimpleGoalChecker](15_simple_goal_checker.md) 相同：XY（可 stateful）+ 航向。

### Step 2：线速度停止

$$
v_{trans} = \sqrt{v_x^2 + v_y^2} \leq v_{trans}^{stop}
$$

### Step 3：角速度停止

$$
|\omega_z| \leq \omega_{rot}^{stop}
$$

### Step 4：综合

Step 1 ∧ Step 2 ∧ Step 3 均满足 → **到达**。

---

## 3. 默认参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `trans_stopped_velocity_` | 0.25 m/s | 线速度上限 |
| `rot_stopped_velocity_` | 0.25 rad/s | 角速度上限 |
| （继承）`xy_goal_tolerance_` | 0.25 m | |
| （继承）`yaw_goal_tolerance_` | 0.25 rad | |

---

## 4. 适用场景

| 场景 | 原因 |
|------|------|
| 充电对接 | 必须静止才能插入 |
| 电梯进出 | 防止惯性滑移 |
| 精密装配 | 到位后不容许微动 |
| 乘梯/载人 | 安全规范要求停稳 |

---

## 5. 调参建议

| 场景 | v_trans_stop | ω_rot_stop |
|------|--------------|------------|
| 通用室内 | 0.25 m/s | 0.25 rad/s |
| 严格对接 | 0.05 m/s | 0.1 rad/s |
| 粗糙地面 | 0.35 m/s | 0.35 rad/s（避免振动误判） |

配合 `VelocitySmoother` 可更快满足停止条件。

---

## 6. GetTolerances

在 `SimpleGoalChecker::GetTolerances` 基础上覆盖速度容差：

- `vel_tolerance.linear.x/y` = `trans_stopped_velocity_`
- `vel_tolerance.angular.z` = `rot_stopped_velocity_`

---

## 7. 源码索引

```cpp
// autonomy/control/checker/stopped_goal_checker.cpp:44-57
bool StoppedGoalChecker::IsGoalReached(...) {
  if (!SimpleGoalChecker::IsGoalReached(...)) return false;
  return fabs(velocity.angular.z) <= rot_stopped_velocity_
      && hypot(velocity.linear.x, velocity.linear.y) <= trans_stopped_velocity_;
}
```

---

## 8. 参考文献

1. Nav2 StoppedGoalChecker: [controller server plugins](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)
