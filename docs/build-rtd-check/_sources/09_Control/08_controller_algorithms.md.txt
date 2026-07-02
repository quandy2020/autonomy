# 8. 局部控制器算法

本文详述 Autonomy 配置中预留的局部控制器算法原理、数学公式与参数说明。当前仓库**尚无具体插件实现**，但 `controller.lua` 已包含完整配置模板，几何工具函数（lookahead、圆-线段交点）已就绪。

> 数学推导见 [03_math.md](03_math.md)；综述对比见 [09_survey.md](09_survey.md)。

## 8.1 算法总览

| 算法 | 类型 | 配置 id | 实现状态 |
|------|------|---------|----------|
| Graceful Controller | 几何 + 平滑 | `graceful_controller` | ⏳ 配置预留 |
| MPPI Controller | 随机 MPC | `mppi_controller` | ⏳ 配置预留 |
| Regulated Pure Pursuit | 几何 + 调节 | — | ❌ 未配置 |
| DWB | 速度空间采样 | — | ❌ 未配置 |
| TEB | 时空优化 | — | ❌ 未配置 |

## 8.2 Graceful Controller

Nav2 `nav2_graceful_controller` 的平滑路径跟踪器，特点是在接近目标时减速、支持初始旋转对齐、可选碰撞检测。

### 8.2.1 控制流程

```
1. 路径变换到机器人坐标系
2. 若 initial_rotation 且航向误差大 → 原地旋转
3. 计算动态 lookahead 距离 L_d ∈ [min_lookahead, max_lookahead]
4. 沿路径找 lookahead 点（GetLookAheadPoint）
5. 计算曲率 κ → ω_z = κ · v_x
6. 根据 slowdown_radius 调节线速度
7. 若 use_collision_detection → 沿弧检测 costmap 碰撞
8. 输出 (v_x, ω_z)
```

### 8.2.2 Lookahead 距离

动态 lookahead 随速度变化：

$$
L_d = \operatorname{clamp}\big(\alpha \cdot |v_x|,\; L_{min},\; L_{max}\big)
$$

Autonomy 配置：

| 参数 | 值 | 说明 |
|------|-----|------|
| `max_lookahead` | 0.55 m | 最大前瞻 |
| `min_lookahead` | 0.25 m | 最小前瞻 |
| `v_linear_max` | 0.5 m/s | 最大线速度 |
| `v_linear_min` | 0.05 m/s | 最小线速度 |
| `v_angular_max` | 1.0 rad/s | 最大角速度 |

### 8.2.3 减速区

距目标 $d_{goal} < r_{slowdown}$ 时线速度线性衰减：

$$
v_x = v_{min} + (v_{max} - v_{min}) \cdot \frac{d_{goal}}{r_{slowdown}}
$$

配置 `slowdown_radius = 0.5` m。

### 8.2.4 初始旋转

当 `initial_rotation = true` 且 $|\mathrm{AngleDiff}(\theta, \theta_{path})| > \theta_{threshold}$ 时，先原地旋转对齐路径方向，再开始跟踪。

### 8.2.5 碰撞检测

`use_collision_detection = true` 时，沿预测弧采样点检测 costmap：

$$
p(s) = (x + s\cos\theta,\; y + s\sin\theta), \quad s \in [0, L_d]
$$

若任一点代价 ≥ `INSCRIBED`（253），拒绝该命令并减速。

### 8.2.6 配置示例

```lua
graceful_controller = {
    transform_tolerance = 0.1,
    max_lookahead = 0.55,
    min_lookahead = 0.25,
    v_linear_max = 0.5,
    v_linear_min = 0.05,
    v_angular_max = 1.0,
    slowdown_radius = 0.5,
    initial_rotation = true,
    allow_backward = false,
    use_collision_detection = true,
},
```

## 8.3 MPPI Controller

Nav2 `nav2_mppi_controller` 基于 **Model Predictive Path Integral Control**，通过大量随机采样控制序列，用 softmax 加权得到最优控制。

### 8.3.1 算法框架

```
for each control cycle:
  1. 从当前 (x, y, θ, v) 出发
  2. 采样 K=batch_size 条控制序列 U^{(k)} = {u_0, ..., u_{H-1}}
  3. 前向仿真 H=time_steps 步，步长 dt=model_dt
  4. 计算每条轨迹总代价 S^{(k)} = Σ critics
  5. softmax 权重 w_k = exp(-(S^{(k)} - S_min) / λ) / Σ
  6. 输出 u_0* = Σ w_k · u_0^{(k)}
  7. 控制序列移位（shift）供下一周期 warm-start
```

### 8.3.2 运动模型

**DiffDrive**（配置 `motion_model = "DiffDrive"`）：

$$
\begin{aligned}
x_{t+1} &= x_t + v_x \cos\theta_t \cdot dt \\
y_{t+1} &= y_t + v_x \sin\theta_t \cdot dt \\
\theta_{t+1} &= \theta_t + \omega_z \cdot dt
\end{aligned}
$$

**Ackermann** 额外约束：

$$
|\omega_z| \leq \frac{|v_x|}{R_{\min}}, \quad R_{\min} = 0.2\,\mathrm{m}
$$

### 8.3.3 采样分布

控制扰动从高斯分布采样：

$$
u_t^{(k)} = \bar{u}_t + \mathcal{N}(0, \Sigma), \quad
\Sigma = \mathrm{diag}(\sigma_{v_x}^2, \sigma_{v_y}^2, \sigma_{\omega_z}^2)
$$

| 参数 | 值 |
|------|-----|
| `vx_std` | 0.2 |
| `vy_std` | 0.2 |
| `wz_std` | 0.4 |
| `batch_size` | 2000 |
| `time_steps` | 56 |
| `model_dt` | 0.05 s |
| `iteration_count` | 1 |

预测时域：$T = H \cdot dt = 56 \times 0.05 = 2.8$ s。

### 8.3.4 代价函数

总代价：

$$
S^{(k)} = \sum_{t=0}^{H-1} \sum_{c \in \mathcal{C}_{\mathrm{critic}}} w_c \cdot \mathcal{L}_c(x_t^{(k)}, u_t^{(k)})
+ \frac{\gamma}{2} u_t^{(k)\top} \Sigma^{-1} u_t^{(k)}
$$

| 参数 | 值 | 含义 |
|------|-----|------|
| `temperature` | 0.3 | softmax 温度 $\lambda$ |
| `gamma` | 0.015 | 控制代价权重 |

### 8.3.5 Critics 详解

#### ConstraintCritic

惩罚超出速度/加速度限制的采样：

$$
\mathcal{L}_{constraint} = \sum_t \Big[ \max(0, |v_x| - v_{x,max})^2 + \max(0, |\omega_z| - \omega_{max})^2 \Big]
$$

权重 `cost_weight = 4.0`。

#### CostCritic

沿轨迹采样 costmap 代价：

$$
\mathcal{L}_{cost} = \sum_t c(x_t, y_t), \quad c \geq c_{\mathrm{crit}} \Rightarrow \mathrm{invalid\ trajectory}
$$

其中 $c_{\mathrm{crit}}$ 为 `critical_cost` 阈值。

| 参数 | 值 |
|------|-----|
| `cost_weight` | 3.81 |
| `critical_cost` | 300.0 |
| `collision_cost` | 1,000,000 |
| `trajectory_point_step` | 2 |

#### GoalCritic

距终点欧氏距离，仅在 `threshold_to_consider = 1.4` m 内激活：

$$
\mathcal{L}_{goal} = w \cdot \|p_t - p_{goal}\|_2
$$

#### GoalAngleCritic

终点航向误差，在 0.5 m 内激活：

$$
\mathcal{L}_{angle} = w \cdot |\mathrm{AngleDiff}(\theta_t, \theta_{goal})|
$$

#### PathAlignCritic

轨迹点到全局路径的最短距离：

$$
\mathcal{L}_{align} = w \cdot \min_{p \in \mathcal{P}} \|p_t - p\|_2
$$

`offset_from_furthest = 20` 表示从路径最远到达点开始评估。

#### PathFollowCritic

鼓励沿路径前进，评估轨迹最远到达点与路径上对应点的距离。

#### PathAngleCritic

轨迹方向与路径切向的夹角：

$$
\mathcal{L}_{\mathrm{path}} = w \cdot |\mathrm{AngleDiff}(\theta_t, \theta_{\mathrm{path}})|
$$

#### PreferForwardCritic

惩罚后退：

$$
\mathcal{L}_{forward} = w \cdot \max(0, -v_x)
$$

### 8.3.6 频率对齐

```
controller_frequency = 20 Hz  ↔  model_dt = 0.05 s
```

两者必须满足 $f_{ctrl} = 1 / dt$，否则控制序列移位会错位。

### 8.3.7 配置示例

完整配置见 `config/control/controller.lua` 中 `mppi_controller` 段。启用方式：

```lua
controller_plugins = {
    "mppi_controller:MppiController",
},
```

## 8.4 Regulated Pure Pursuit（RPP）

Nav2 默认局部控制器，Pure Pursuit 的增强版。Autonomy 尚未配置，但工具函数已支持核心几何计算。

### 8.4.1 基础 Pure Pursuit

在机器人坐标系中，lookahead 点 $(x^*, y^*)$，曲率：

$$
\kappa = \frac{2 y^*}{L_d^2}
$$

$$
\omega_z = \kappa \cdot v_x, \quad v_x = v_{desired}
$$

### 8.4.2 Regulated 调节

RPP 在 PP 基础上增加三项调节：

**1. 曲率调节**（防止过急转弯）：

$$
v_x = \min\!\left(v_{desired},\; \frac{\omega_{max}}{|\kappa|}\right)
$$

**2. 障碍调节**（接近障碍减速）：

$$
v_x = \min\!\left(v_x,\; \alpha \cdot d_{obstacle}\right)
$$

**3. 接近目标减速**：

$$
v_x = \min\!\left(v_x,\; \beta \cdot d_{goal}\right)
$$

### 8.4.3 Autonomy 已有工具

| 函数 | 文件 | 用途 |
|------|------|------|
| `GetLookAheadPoint()` | `controller_utils.cpp` | 沿路径找 lookahead |
| `CircleSegmentIntersection()` | `controller_utils.cpp` | 圆-线段交点 |
| `LinearInterpolation()` | `controller_utils.cpp` | 路径段插值 |

## 8.5 DWB（Dynamic Window Benchmark）

Nav2 经典局部规划器，在 $(v, \omega)$ 速度空间中采样并评分。

### 8.5.1 动态窗口

$$
\mathcal{V}_d = \left\{ (v, \omega) :
\begin{aligned}
& v \in [v_c - a_{max}\Delta t,\; v_c + a_{max}\Delta t] \cap [v_{min}, v_{max}] \\
& \omega \in [\omega_c - \dot\omega_{max}\Delta t,\; \omega_c + \dot\omega_{max}\Delta t]
\end{aligned}
\right\}
$$

### 8.5.2 评分函数

$$
G(v, \omega) = \sum_i w_i \cdot \mathcal{S}_i(v, \omega)
$$

| Critic | 含义 |
|--------|------|
| BaseObstacle | 轨迹最近障碍距离 |
| PathAlign | 与路径对齐 |
| PathDist | 距路径距离 |
| GoalDist | 距目标距离 |
| GoalAlign | 目标方向对齐 |
| PreferForward | 偏好前进 |
| RotateToGoal | 接近目标时旋转 |
| Oscillation | 抑制振荡 |
| Twirling | 抑制原地旋转 |

## 8.6 控制器对比

| 维度 | Graceful | MPPI | RPP | DWB |
|------|----------|------|-----|-----|
| 计算量 | 低 | 高 | 低 | 中 |
| 动态避障 | 中（碰撞检测） | 强 | 弱 | 强 |
| 参数数量 | 少 | 多 | 中 | 多 |
| 路径跟踪精度 | 高 | 高 | 中 | 中 |
| 非完整约束 | ✅ | ✅ | ✅ | ✅ |
| 全向 | ❌ | ✅ | ❌ | 可选 |

## 8.7 实现路线建议

1. **Phase 1**：基于已有 `controller_utils` 实现 Regulated Pure Pursuit（最低成本、可快速验证 FollowPath 循环）
2. **Phase 2**：移植 Graceful Controller（配置已就绪）
3. **Phase 3**：引入 MPPI（计算密集，建议可选编译）
4. **Phase 4**：评估 DWB / TEB 需求

## 8.8 自定义控制器开发清单

```
□ 继承 ControllerInterface
□ 实现 ComputeVelocityCommands()
□ 实现 SetPlan() — 接收 planning_msgs::Path
□ 实现 IsGoalReached() — 委托 goal_checker 或内部判定
□ 实现 SetSpeedLimit()
□ 注册 AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN
□ 编写 controller.lua 配置段
□ 单元测试：直线路径、急转弯、障碍场景
```
