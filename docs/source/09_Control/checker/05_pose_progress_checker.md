(pose-progress-checker)=
# 5. PoseProgressChecker

继承 SimpleProgressChecker，把 **航向变化** 也算作进度。源码：`checker/pose_progress_checker.*`。

在 SimpleProgress 之上，把航向变化也算有效位移，刷新进度基准。接近目标前需大角度原地转向、XY 几乎不动时用；**Server 未默认加载**，需自行注册。

## 判据

相对基准位姿，满足任一即视为有进度并刷新基准：

- 平面位移 $\ge r_{\mathrm{req}}$，或
- $|\Delta\theta|\ge\varepsilon_\theta$（相对基准航向）

超时逻辑与 SimpleProgress 相同（`movement_time_allowance`）。

## 何时用

- 接近目标前需要 **大角度原地转向**，XY 位移很小。
- SimpleProgress 频繁 `FailedToMakeProgress`，但 RViz 里机器人在正常转圈。

**注意：** Server **未** 默认加载；需自行注册。

## 参数

与 SimpleProgress 共用：`required_movement_radius`、`movement_time_allowance`，外加航向阈值（见源码 `PoseProgressChecker::IsRobotMovedEnough`）。

## 常见坑

| 现象 | 说明 |
|------|------|
| 打滑空转也算进度 | 航向变了即刷新；结合里程计/视觉需上层判断 |
| 仍报卡住 | 角速度太慢，$\Delta\theta$ 长期不够；调小航向阈值或增大时限 |
