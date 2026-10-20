# 3. VelocitySmoother

对 `cmd_vel` 做加速度限幅和 deadband，思路接近 nav2_velocity_smoother。库在 `utils/velocity_smoother.*`，proto 在 `smoother_options.proto`，细节见 [smoother/01](smoother/01_velocity_smoother_impl.md)。

注意：控制环里还没挂上这个类。`ControllerServer` 里用的是 `OdomSmoother`（平滑里程计），不是输出速度平滑。

$$
v_{\mathrm{out}}=v_{\mathrm{curr}}+\mathrm{clamp}(\eta\cdot\Delta v,\,a_{\min}\Delta t,\,a_{\max}\Delta t)
$$

```cpp
VelocitySmootherOptions opts;
opts.set_smoothing_frequency(20.0);
opts.set_scale_velocities(true);
opts.add_max_velocity(0.5);
opts.add_max_velocity(0.0);
opts.add_max_velocity(2.5);
VelocitySmoother smoother(opts);
```

常用字段：`max_velocity` / `max_accel` / `max_decel`、`smoothing_frequency`、`scale_velocities`。
