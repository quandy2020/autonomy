(simple-progress-checker)=
# 4. SimpleProgressChecker

**默认** Progress checker：在一段时间内，机器人相对基准位姿的 **XY 位移** 必须超过半径，否则判无进度。源码：`checker/simple_progress_checker.*`。

在 `movement_time_allowance` 内，相对基准位姿的 XY 位移须超过 `required_movement_radius`，否则抛 `FailedToMakeProgress`。**默认 Progress checker**；机器人在障前原地打转接近目标时可能误报卡住。

## 判据

- 维护基准 pose 与时间戳。
- 若尚未设基准，或 $\|p-p_{\mathrm{base}}\|>r_{\mathrm{req}}$：刷新基准，返回 true（有进度）。
- 否则若 elapsed $\le$ `movement_time_allowance`：返回 true（仍在宽限内）。
- 超时且位移不足：返回 false → Server 抛 `FailedToMakeProgress`。

实现已包含 `movement_time_allowance`（Nav2 语义），由 `SetMovementTimeAllowance` / proto 配置。

## 参数

| 参数 | 默认 |
|------|------|
| `required_movement_radius` | 0.5 m |
| `movement_time_allowance` | 10 s |

`Reset()` 在每次 FollowPath 开始时由 Server 调用。

## 何时用

一般差速/全向跟路径。机器人在障碍前 **原地打转接近** 时，位移可能长期 $<r$，会误报卡住 → 换 [PoseProgressChecker](05_pose_progress_checker.md)（需注册）。

## 常见坑

| 现象 | 处理 |
|------|------|
| 刚起步就失败 | 半径过大、时限过短；或 footprint 顶障根本动不了 |
| 堵死不动不失败 | 时限过长；或一直在微动超过半径 |
| 窄缝反复微调 | 略减 `required_movement_radius` |
