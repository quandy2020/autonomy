(velocity-smoother-impl)=
# 1. VelocitySmoother 实现

归属 [§4](../03_velocity_smoother.md)。源码：`control/utils/velocity_smoother.*`。

### 定义

每轴：$v_{\mathrm{out}}=v_{\mathrm{curr}}+\mathrm{clamp}(\Delta v,\,a_{\min}\Delta t,\,a_{\max}\Delta t)$。`scale_velocities` 时按最严轴缩放，近似保曲率。deadband 抑制微抖。

### 结构

`VelocitySmootherOptions`（proto）→ 构造 → 输入命令 → `smootherTimer()` / 等效步进输出。

### 用法

见 [§4](../03_velocity_smoother.md)。节点定时发布接线 。

关键字段：`smoothing_frequency`、`max_velocity[3]`、`max_accel`/`max_decel`、`scale_velocities`、deadband。
