(velocity-smoother-impl)=
# 1. VelocitySmoother 实现

库类 `VelocitySmoother`，配置 `VelocitySmootherOptions`（proto）。总览 [03](../03_velocity_smoother.md)。

对待发 `cmd_vel` 各分量做加速度限幅，可选统一 scale；与订阅 Odometry 的 OdomSmoother 不同。**当前 FollowPath 未默认接入** VelocitySmoother，可在 `PublishVelocity` 前或独立节点做 raw→smooth 话题。

## 算法

对每个速度分量独立做加速度 clamp，再可选 `scale_velocities` 统一缩放。`findEtaConstraint` / `applyConstraints` 在 `velocity_smoother.cpp`。

`inputCommandCallback` / `inputCommandStampedCallback` 接收目标 twist；`smootherTimer()` 按 `smoothing_frequency` 步进输出。

## 和 OdomSmoother 的区别

| | VelocitySmoother | OdomSmoother（Control 内） |
|--|------------------|----------------------------|
| 对象 | 待发 `cmd_vel` | 订阅的 Odometry |
| FollowPath | 未默认接入 | 用于填当前速度给 checker / 控制器 |

## 集成建议

在 `PublishVelocity` 前插入一层，或独立节点订阅 raw cmd、发布 smoothed cmd。频率应 ≥ `controller_frequency`，避免相位 lag 过大。

## 关键 proto 字段

`smoothing_frequency`、`max_velocity`（3 元）、`max_accel`、`max_decel`、`scale_velocities`、deadband 阈值。

## 常见坑

- 只构造 smoother 不调 `smootherTimer` 或等价步进 → 无输出。
- 首帧 `v_curr` 为零，大指令仍受 accel 限制 → 起步慢是预期行为。
