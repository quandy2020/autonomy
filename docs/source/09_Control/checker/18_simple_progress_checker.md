(simple-progress-checker)=
# 18. SimpleProgressChecker

> 归属 [§3 检查器 · §3.5](../03_checkers.md#35-simpleprogresschecker)
>
> `autonomy::control::checker::SimpleProgressChecker` 检测机器人是否在**时间窗口内产生足够位移**，用于防止 FollowPath 卡住。  
> 公式见 [03_checkers.md §3.12](../03_checkers.md#312-progress-checker-判定数学)。

| 维度 | 说明 |
|------|------|
| 源码 | `autonomy/control/checker/simple_progress_checker.*` |
| 接口 | `common::ProgressChecker` |
| 状态 | ✅ 已实现；`movement_time_allowance` 待实现 |

---

## 1. 语义约定

| 返回值 | 含义 |
|--------|------|
| **true** | 有进度（移动足够或首次调用） |
| **false** | 无进度 → 上层累计后抛 `FailedToMakeProgress` |

> 注意：与 GoalChecker 相反，ProgressChecker 的 **true = 正常**。

---

## 2. 架构

```
每个控制周期:
  Check(current_pose)
      │
      ├─ 首次调用 (baseline_pose_set_ == false)
      │     └─ ResetBaselinePose → return true
      │
      ├─ hypot(x-x_b, y-y_b) > radius_ ?
      │     └─ yes → ResetBaselinePose → return true
      │
      └─ return false  （卡住）
```

---

## 3. 数学原理（Step-by-Step）

### Step 1：转 Pose2D

$$
p = (x,\; y,\; \theta = \mathrm{yaw}(\mathrm{orientation}))
$$

### Step 2：位移量

$$
d = \mathrm{hypot}(x - x_b,\; y - y_b)
$$

### Step 3：判定

$$
d > r \Rightarrow \text{有进度，重置基线}
$$

默认 $r = 0.5$ m（`required_movement_radius`）。

### Step 4：Nav2 时间窗口（待实现）

Nav2 完整语义还包括：在 `movement_time_allowance` 秒内必须满足 Step 3，否则失败。Autonomy 当前**仅看位移**，未计时。

---

## 4. 配置

```lua
progress_checker = {
    required_movement_radius = 0.5,
    movement_time_allowance = 10.0,  -- proto 已定义，C++ 待实现
},
```

---

## 5. 调参建议

| 场景 | radius |
|------|--------|
| 室内通用 | 0.5 m |
| 慢速精细 | 0.1–0.2 m |
| 开阔/高速 | 1.0 m |
| 原地旋转为主 | 改用 [§19 PoseProgressChecker](19_pose_progress_checker.md) |

**过小**：正常慢速移动也会误判卡住。  
**过大**：真正卡住时反应迟钝。

---

## 6. 与 FollowPath 集成

```
progress_checker->Check(pose) == false
    → 累计无进度时间
    → 超过 failure_tolerance / movement_time_allowance
    → throw FailedToMakeProgress
```

---

## 7. 源码索引

```cpp
// autonomy/control/checker/simple_progress_checker.cpp:30-44
bool SimpleProgressChecker::Check(PoseStamped& current_pose) {
  if (!baseline_pose_set_ || IsRobotMovedEnough(current_pose2d)) {
    ResetBaselinePose(current_pose2d);
    return true;
  }
  return false;
}
```

---

## 8. 参考文献

1. Nav2 SimpleProgressChecker: [controller server plugins](https://navigation.ros.org/configuration/packages/configuring-controller-server.html)
