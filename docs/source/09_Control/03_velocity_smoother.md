# 3. VelocitySmoother

对 **输出的** `Twist` 做逐轴加速度限幅和 deadband，思路接近 nav2_velocity_smoother。实现：`autonomy/control/utils/velocity_smoother.*`，proto：`smoother_options.proto`；算法细节 [smoother/01](smoother/01_velocity_smoother_impl.md)。

库已实现但未接入 FollowPath 出口；若要限幅 cmd_vel 需在 Server 外或改 C++ 接线。

## 当前接线状态

**FollowPath 默认不经过此类。** `ControllerServer` 用的是 `OdomSmoother` 平滑 **读进来的里程计**，不是发出去的 `cmd_vel`。若实车抖动，可在驱动前自接一层 VelocitySmoother，或调控制器 / 膨胀 / Progress 时限；不要误以为改 smoother proto 就会生效。

## 什么时候值得接

- 控制器输出阶跃大，驱动器或机械结构对加速度敏感。
- 同一套控制算法要统一限幅，且不想在每个控制器里重复 clamp。

## 核心行为

每轴：

$$
v_{\mathrm{out}}=v_{\mathrm{curr}}+\mathrm{clamp}(\eta\cdot\Delta v,\,a_{\min}\Delta t,\,a_{\max}\Delta t)
$$

`scale_velocities=true` 时按最「紧」的一轴缩放整向量，近似保曲率。deadband 抑制极小指令抖动。

## 关键参数

| 字段 | 作用 |
|------|------|
| `smoothing_frequency` | 定时步进频率，应 ≥ 控制环 |
| `max_velocity[3]` | 线 x/y、角 z 上限 |
| `max_accel` / `max_decel` | 加减速上限 |
| `scale_velocities` | 是否按比例缩放 |
| deadband 相关 | 低于阈值置零 |

```cpp
proto::VelocitySmootherOptions opts;
opts.set_smoothing_frequency(20.0);
opts.set_scale_velocities(true);
opts.add_max_velocity(0.5);
opts.add_max_velocity(0.0);
opts.add_max_velocity(2.5);
VelocitySmoother smoother(opts);
```

## 常见坑

| 现象 | 说明 |
|------|------|
| 改了 conf 仍抖 | Server 未调用 smoother |
| 转弯变慢 | `scale_velocities` 把线速度一起压了 |
| 到点滑过 | 平滑延迟 + Goal 只检位姿不检停稳；停靠用 StoppedGoal（需接线） |
