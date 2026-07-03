(pose-progress-checker)=
# 19. PoseProgressChecker

> 归属 [§3 检查器 · §3.6](../03_checkers.md#36-poseprogresschecker)
>
> `autonomy::control::checker::PoseProgressChecker` 继承 `SimpleProgressChecker`，将**航向变化**也视为有效进度。

| 维度 | 说明 |
|------|------|
| 源码 | `autonomy/control/checker/pose_progress_checker.*` |
| 基类 | `SimpleProgressChecker` |
| 状态 | ✅ 已实现 |

---

## 1. 架构

```
Check(current_pose)
    │
    ├─ 首次 or IsRobotMovedEnough(pose2d) ?
    │     │
    │     ├─ hypot(x-x_b, y-y_b) > radius_     （继承）
    │     └─ |NormalizeAngleDiff(θ-θ_b)| > Δθ   （扩展）
    │
    └─ 任一满足 → ResetBaselinePose → true
```

---

## 2. 数学原理（Step-by-Step）

### Step 1：XY 位移（同 Simple）

$$
d_{xy} = \mathrm{hypot}(x - x_b,\; y - y_b) > r
$$

### Step 2：航向变化

$$
\Delta\theta = \left|\mathrm{NormalizeAngleDiff}(\theta - \theta_b)\right| > \Delta\theta_{req}
$$

默认 $\Delta\theta_{req} = 0.5$ rad。

### Step 3：逻辑或

$$
\text{有进度} \Leftrightarrow d_{xy} > r \;\lor\; \Delta\theta > \Delta\theta_{req}
$$

---

## 3. 为何需要 PoseProgressChecker

| 情况 | SimpleProgressChecker | PoseProgressChecker |
|------|----------------------|---------------------|
| 原地旋转对齐 | ❌ 判为无进度 | ✅ 转角算进度 |
| 先转向再前进 | 可能误报卡住 | ✅ |
| 纯直线行驶 | ✅ | ✅ |

---

## 4. 默认参数

| 参数 | 默认 |
|------|------|
| `radius_` | 0.5 m（继承） |
| `required_movement_angle_` | 0.5 rad |

---

## 5. 调参建议

| 场景 | radius | Δθ_req |
|------|--------|--------|
| 初始旋转策略 | 0.5 m | 0.3 rad |
| 窄通道直线 | 0.3 m | 0.8 rad（减少误触发） |
| 全向/高旋转 | 0.5 m | 0.2 rad |

---

## 6. 源码索引

```cpp
// autonomy/control/checker/pose_progress_checker.cpp:50-54
bool PoseProgressChecker::IsRobotMovedEnough(const Pose2D& pose) {
  return PoseDistance(pose, baseline_pose_) > radius_
      || PoseAngleDistance(pose, baseline_pose_) > required_movement_angle_;
}
```

---

## 7. 与 SimpleProgressChecker 选型

```
机器人会先原地转较大角度？
  ├─ 是 → PoseProgressChecker
  └─ 否 → SimpleProgressChecker（更简单）
```

---

## 8. 参考文献

1. Nav2 PoseProgressChecker: [controller server plugins](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)
