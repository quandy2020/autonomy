(graceful-controller)=
# 1. Graceful Controller

归属 [§4](../04_controller_algorithms.md)。Park & Kuipers 平滑律；源码 `graceful_controller/` 。

### 定义

差速在自我中心极坐标 $(r,\phi,\delta)$ 跟踪 motion target：

$$
\dot r=-v\cos\delta,\quad
\dot\phi=\frac{v}{r}\sin\delta,\quad
\dot\delta=\frac{v}{r}\sin\delta+\omega
$$

$\delta_{\mathrm{ref}}=\arctan(-k_1\phi)$（Nav2 `k_phi`）；曲率/角速度闭式（ICRA (13)(14)）；$v$ 经曲率与 `slowdown_radius` 调节。

### 结构

路径上取 lookahead motion target → 极坐标误差 → 闭式 $(v,\omega)$ → 可选 costmap 轨迹仿真拒不可行 target。

### 用法 / 调参

| 现象 | 调整 |
|------|------|
| 切弯穿墙 | 开碰撞仿真；减小 `max_lookahead` |
| 近目标 overshoot | 增大 `slowdown_radius` |
| 起步摆头 | `initial_rotation` |
