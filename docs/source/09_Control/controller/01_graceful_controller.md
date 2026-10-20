(graceful-controller)=
# 1. Graceful Controller

Park & Kuipers 平滑跟踪律，差速底盘在极坐标 $(r,\phi,\delta)$ 下跟踪路径上的 motion target。源码：`graceful_controller/`。 Park–Kuipers 在极坐标 (r, φ, δ) 下对 motion target 做平滑跟踪律，算力低于采样类控制器。适合室内跟全局 Path、想要比 Pure Pursuit 更柔的弯内行为；动态障密或需多 critic 时换 MPPI。

## 何时用

算力有限、想要比 Pure Pursuit 更平滑的弯内行为；室内跟全局 Path。动态障多、需要采样 MPC 时换 MPPI。

## 控制律（摘要）

$$
\dot r=-v\cos\delta,\quad
\dot\phi=\frac{v}{r}\sin\delta,\quad
\dot\delta=\frac{v}{r}\sin\delta+\omega
$$

$\delta_{\mathrm{ref}}=\arctan(-k_1\phi)$；曲率与 $\omega$ 有闭式；$v$ 随曲率与 `slowdown_radius` 缩小。可选 costmap 轨迹仿真，拒绝会碰撞的 target。

## 调参

| 现象 | 调整 |
|------|------|
| 切弯穿墙 | 开碰撞仿真；减小 `max_lookahead` |
| 近目标 overshoot | 增大 `slowdown_radius` |
| 起步摆头 | `initial_rotation` 相关项 |

## 常见坑

- 无 costmap 时碰撞仿真不起作用，窄道仍可能贴障。
- 与 SimpleGoal 联用时，近终点减速依赖控制器自身，不是 StoppedGoal。

归属 [04 算法总览](../04_controller_algorithms.md)。
