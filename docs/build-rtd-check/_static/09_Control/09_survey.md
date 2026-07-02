# 9. 运动控制算法综述（Survey）

本文从**学术发展史、算法体系、工程实践、影响因素**四个维度，系统综述移动机器人局部运动控制（Local Control / Trajectory Tracking）领域，并明确 Autonomy `control` 模块在其中的定位、能力边界与选型依据。

> 公式推导见 [Control 指南 · 数学原理](03_math.md)；实现细节见 [架构设计](05_architecture.md) 与各子文档。

---

## 9.1 概述：运动控制在导航栈中的位置

移动机器人导航（Navigation）通常分解为四个耦合子问题：

| 子问题 | 英文 | 典型模块 | 频率 |
|--------|------|----------|------|
| 定位 | Localization | `localization` | 10–50 Hz |
| 建图 | Mapping | `map` | 1–10 Hz |
| 路径规划 | Planning | `planning` | 1–5 Hz |
| **运动控制** | **Control** | **`control`** | **10–50 Hz** |

**局部运动控制**（Local Control）接收全局路径，以固定频率输出速度命令 $u = (v_x, v_y, \omega_z)$，使机器人沿路径运动并到达目标。Autonomy `control` 模块专注此层，对齐 Nav2 `nav2_controller`。

```
Planning.Path ──→ Local Controller ──→ VelocitySmoother ──→ cmd_vel ──→ 底盘
                       ↑                      ↑
                  local costmap            odom 反馈
                  (障碍/膨胀)            (CLOSED_LOOP)
```

---

## 9.2 问题形式化

### 9.2.1 状态与控制

机器人在 $SE(2)$ 或 $SE(2) \times \mathbb{R}^n$ 上运动：

$$
x = (x, y, \theta, v_x, v_y, \omega_z)^\top, \quad
u = (v_x^{cmd}, v_y^{cmd}, \omega_z^{cmd})^\top
$$

给定参考路径 $\mathcal{P} = \{p_0, \ldots, p_N\}$，$p_i \in SE(2)$，求控制律 $u = \pi(x, \mathcal{P}, t)$ 使：

$$
\min_{\pi} \int_0^T \Big( \|e_p\|_Q^2 + e_\theta^2 + u^\top R u \Big) dt
$$

约束 $(x,y) \in \mathcal{C}_{free}$，$|v| \leq v_{max}$，$|a| \leq a_{max}$。

### 9.2.2 误差定义

**全局误差**：

$$
e_p = \begin{bmatrix} x - x_g \\ y - y_g \end{bmatrix}, \quad
e_\theta = \mathrm{AngleDiff}(\theta, \theta_g)
$$

**Frenet 误差**（沿路径投影）：

$$
e_y = d_{\mathrm{signed}}(q, \mathcal{P}), \quad e_s = s - s_{\mathrm{ref}}
$$

其中 $e_y$ 为点到路径的有符号距离，$e_s$ 为弧长误差。

### 9.2.3 约束类型

| 约束 | 说明 | 控制器是否考虑 |
|------|------|----------------|
| 几何约束 | 不碰撞 | ✅ 所有控制器 |
| 非完整约束 | $v_y = 0$（DiffDrive） | ✅ |
| 运动学约束 | 最小转弯半径 | ✅ RPP / MPPI |
| 动力学约束 | 加速度限制 | ✅ VelocitySmoother |
| 实时约束 | 计算时间 < 控制周期 | ✅ 采样类 |

---

## 9.3 发展时间轴

<div class="planning-timeline-v2">

<div class="timeline-era-block era-foundation">
  <div class="timeline-era-header">奠基期 · 1980s–1990s</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1985</div>
      <div class="timeline-milestone-title">Pure Pursuit</div>
      <div class="timeline-milestone-desc">几何路径跟踪，lookahead 思想</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1995</div>
      <div class="timeline-milestone-title">Dynamic Window</div>
      <div class="timeline-milestone-desc">Fox et al. 速度空间采样避障</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1995</div>
      <div class="timeline-milestone-title">Stanley</div>
      <div class="timeline-milestone-desc">DARPA 自动驾驶横向控制</div>
    </div>
  </div>
</div>

<div class="timeline-era-block era-classical">
  <div class="timeline-era-header">经典期 · 2000s–2010s</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2009</div>
      <div class="timeline-milestone-title">TEB</div>
      <div class="timeline-milestone-desc">Timed Elastic Band 时空优化</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2010</div>
      <div class="timeline-milestone-title">DWA → DWB</div>
      <div class="timeline-milestone-desc">ROS navigation 标准化</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2018</div>
      <div class="timeline-milestone-title">Regulated PP</div>
      <div class="timeline-milestone-desc">Nav2 默认控制器</div>
    </div>
  </div>
</div>

<div class="timeline-era-block era-modern">
  <div class="timeline-era-header">现代期 · 2020s</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2022</div>
      <div class="timeline-milestone-title">MPPI</div>
      <div class="timeline-milestone-desc">Nav2 集成随机 MPC 控制器</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2023</div>
      <div class="timeline-milestone-title">Graceful Controller</div>
      <div class="timeline-milestone-desc">Nav2 平滑跟踪控制器</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2024</div>
      <div class="timeline-milestone-title">Autonomy control</div>
      <div class="timeline-milestone-desc">Nav2 架构移植，骨架 + Checker/Smoother</div>
    </div>
  </div>
</div>

</div>

---

## 9.4 算法分类体系

### 9.4.1 按方法类型

| 类别 | 代表算法 | 核心思想 | 计算复杂度 |
|------|----------|----------|------------|
| **几何跟踪** | Pure Pursuit, Stanley, Graceful | 几何关系直接映射到 $(v, \omega)$ | $O(n)$ |
| **速度空间采样** | DWA, DWB | 在 $(v, \omega)$ 空间采样评分 | $O(n \cdot m)$ |
| **随机 MPC** | MPPI | 大量采样控制序列 + softmax | $O(K \cdot H)$ |
| **优化方法** | TEB, CHOMP | 非线性优化轨迹 | $O(n^2 \sim n^3)$ |
| **反馈线性化** | Backstepping, FL | 非线性系统线性化控制 | $O(1)$ |
| **Learning-based** | End-to-end, RL | 数据驱动策略 | 推理 $O(1)$ |

### 9.4.2 按感知依赖

| 类型 | 需要 costmap | 需要 odom | 代表 |
|------|-------------|-----------|------|
| 纯跟踪 | ❌ | ✅ | Pure Pursuit |
| 障碍感知 | ✅ | ✅ | DWB, MPPI |
| 预测性 | ✅ + 预测 | ✅ | TEB + prediction |

---

## 9.5 核心算法详解

### 9.5.1 Pure Pursuit

**思想**：在路径上选取距机器人 $L_d$ 处的 lookahead 点，计算使机器人朝向该点的曲率。

$$
\kappa = \frac{2 y^*}{L_d^2}, \quad \omega = \kappa v
$$

| 优点 | 缺点 |
|------|------|
| 极简、低延迟 | 不避障 |
| 参数少（仅 $L_d$） | 急转弯切弯 |
| 工业验证充分 | 无速度调节 |

### 9.5.2 Regulated Pure Pursuit

Nav2 在 PP 基础上增加**曲率/障碍/目标**三项速度调节。Autonomy 工具函数已支持 PP 几何核心。

### 9.5.3 Stanley Controller

$$
\delta = e_\theta + \arctan\!\left(\frac{k_e e_y}{v + \varepsilon}\right)
$$

横向误差 $e_y$ 与航向误差 $e_\theta$ 联合控制。适合高速场景（自动驾驶），室内 AGV 较少使用。

### 9.5.4 DWB

在动态窗口 $\mathcal{V}_d$ 内均匀采样 $(v, \omega)$，前向仿真短轨迹，多 critic 加权评分选最优。

$$
G(v, \omega) = \sum_i w_i \mathcal{S}_i(v, \omega)
$$

| 优点 | 缺点 |
|------|------|
| 成熟、可_recritic 可组合 | 采样密度与分辨率权衡 |
| 实时避障 | 高维速度空间扩展困难 |
| Nav2 生态完善 | 参数调优复杂 |

### 9.5.5 MPPI

**Model Predictive Path Integral Control** 基于信息论优化：

$$
u^* = \frac{\int u \cdot \exp(-S(u)/\lambda) \, du}{\int \exp(-S(u)/\lambda) \, du}
 \approx \sum_k w_k u^{(k)}
$$

| 优点 | 缺点 |
|------|------|
| 天然处理非线性/约束 | 计算量大（batch_size × time_steps） |
| 多 critic 灵活组合 | 需 GPU/多核才能高频率 |
| 支持全向/Ackermann | 参数敏感（temperature, gamma） |

Autonomy `controller.lua` 已配置完整 MPPI 参数（56 步 × 2000 采样）。

### 9.5.6 TEB

将路径建模为**弹性带**（一系列 pose + 时间间隔），优化目标：

$$
\min \sum_i \Big( w_t \Delta t_i^2 + w_o d_{obs,i}^2 + w_v (v_i - v_{pref})^2 \Big)
$$

适合动态环境，但计算量较大，Autonomy 暂未集成。

### 9.5.7 Graceful Controller

Nav2 2023 引入，强调**平滑性**：

- 初始旋转对齐
- 减速区（slowdown_radius）
- 可选碰撞检测
- 不允许后退（可配置）

Autonomy 配置已就绪，适合室内差速机器人。

---

## 9.6 辅助组件

### 9.6.1 Goal Checker

判定导航是否完成。Autonomy 已实现三种（Simple / Position / Stopped），详见 [06_checkers.md](06_checkers.md)。

### 9.6.2 Progress Checker

检测机器人是否"卡住"。Simple 版仅看位移；Pose 版还看转角变化。

### 9.6.3 VelocitySmoother

Nav2 独立节点，Autonomy 算法已移植。加速度约束 + deadband + 开/闭环反馈。详见 [07_velocity_smoother.md](07_velocity_smoother.md)。

---

## 9.7 Autonomy control 模块定位

### 9.7.1 已实现

| 组件 | 完成度 |
|------|--------|
| ControllerInterface 插件接口 | ✅ |
| Goal Checker ×3 | ✅ |
| Progress Checker ×2 | ✅ |
| VelocitySmoother 算法 | ✅ |
| OdomSmoother 滑动平均 | ✅ |
| 几何工具（lookahead、圆-线段交点） | ✅ |
| Lua → ControllerOptions | ✅ |
| ControllerServer 骨架 | ✅ 部分 |

### 9.7.2 待实现

| 组件 | 优先级 |
|------|--------|
| FollowPath 控制循环 | P0 |
| 插件加载（Autolink） | P0 |
| 首个控制器插件（RPP/Graceful） | P1 |
| CheckerOptions Lua 接线 | P1 |
| MPPI 插件 | P2 |
| VelocitySmoother 节点接线 | P2 |

### 9.7.3 与 Nav2 能力矩阵

| 能力 | Nav2 | Autonomy |
|------|------|----------|
| FollowPath Action | ✅ | ⏳ |
| Regulated PP | ✅ | ❌ |
| DWB | ✅ | ❌ |
| MPPI | ✅ | ⏳ 配置 |
| Graceful | ✅ | ⏳ 配置 |
| Goal Checker | ✅ | ✅ |
| Progress Checker | ✅ | ✅（缺时间窗口） |
| Velocity Smoother | ✅ | ✅（未接线） |
| 插件动态加载 | ✅ | ⏳ |

---

## 9.8 影响因素

### 9.8.1 控制频率

| 频率 | 适用 | 说明 |
|------|------|------|
| 10 Hz | 低算力 / 慢速 | 延迟较大 |
| 20 Hz | **推荐默认** | 与 MPPI dt=0.05 对齐 |
| 50 Hz | 高速 / 全向 | 需更强算力 |

### 9.8.2 Lookahead 距离

| $L_d$ | 行为 |
|-------|------|
| 小 (0.2–0.4 m) | 跟踪精确、易振荡 |
| 中 (0.4–0.8 m) | 平衡 |
| 大 (0.8–2.0 m) | 平滑、切弯 |

### 9.8.3 容差与精度

| 参数 | 典型值 | 影响 |
|------|--------|------|
| `xy_goal_tolerance` | 0.05–0.5 m | 到达判定精度 |
| `yaw_goal_tolerance` | 0.1–0.5 rad | 最终朝向精度 |
| `required_movement_radius` | 0.1–1.0 m | 卡住检测灵敏度 |

### 9.8.4 Costmap 分辨率

局部 costmap 默认 0.05 m，rolling window 3×3 m。MPPI CostCritic 沿轨迹采样，分辨率影响碰撞检测精度。

---

## 9.9 算法复杂度对比

| 算法 | 时间复杂度 | 典型耗时 (20Hz) | 内存 |
|------|-----------|----------------|------|
| Pure Pursuit | $O(n)$ | < 0.1 ms | 低 |
| Regulated PP | $O(n)$ | < 0.5 ms | 低 |
| DWB | $O(n_{traj} \cdot n_{sample})$ | 1–5 ms | 中 |
| MPPI (2000×56) | $O(K \cdot H)$ | 5–20 ms | 高 |
| TEB | $O(n^2)$ | 10–50 ms | 中 |
| Graceful | $O(n)$ | < 1 ms | 低 |

---

## 9.10 与其他模块的耦合

```
┌─────────────┐     Path      ┌──────────────────┐
│  planning   │──────────────→│    control       │
└─────────────┘               │                  │
                              │  ControllerServer│
┌─────────────┐   costmap     │       ↓          │
│
│    map      │──────────────→│  ControllerPlugin│
└─────────────┘               │       ↓          │
                              │  cmd_vel         │
┌─────────────┐     odom      └────────┬─────────┘
│ localization│←───────────────────────│
└─────────────┘                        ↓
                              ┌──────────────────┐
                              │  VelocitySmoother│
                              └────────┬─────────┘
                                       ↓
                                    底盘 / 仿真
```

---

## 9.11 未来趋势

| 方向 | 说明 |
|------|------|
| **Learning-based Control** | RL / imitation learning 替代手工 critic |
| **GPU-accelerated MPPI** | CUDA 并行采样，10000+ batch |
| **Unified Planning-Control** | 规划与控制联合优化（如 MPC with full trajectory） |
| **Semantic-aware Control** | 利用语义地图调整行为（人行道减速等） |
| **Multi-robot Coordination** | 分布式控制与冲突消解 |

---

## 9.12 参考文献

1. Coulter, R. (1992). *Implementation of the Pure Pursuit Path Tracking Algorithm*. CMU-RI-TR-92-01.
2. Fox, D., Burgard, W., & Thrun, S. (1997). *The Dynamic Window Approach to Collision Avoidance*. IEEE RAM.
3. Thrun, S., et al. (2006). *Stanley: The Robot that Won the DARPA Grand Challenge*. JFR.
4. Rösmann, C., et al. (2017). *Integrated online trajectory optimization*. IEEE RAM (TEB).
5. Williams, G., et al. (2017). *Information Theoretic MPC for Model-Based Reinforcement Learning*. ICRA (MPPI).
6. Macenski, S., et al. (2022). *Robot Operating System 2: Nav2*. Science Robotics.
7. Navigation2 Documentation: https://docs.nav2.org/

---

## 9.13 术语表

| 术语 | 英文 | 说明 |
|------|------|------|
| 局部规划 | Local Planning | 与 Control 同义，输出速度命令 |
| 轨迹跟踪 | Trajectory Tracking | 沿参考路径运动 |
| Lookahead | Look-ahead Distance | 前瞻距离 $L_d$ |
| 动态窗口 | Dynamic Window | 可达速度集合 $\mathcal{V}_d$ |
| Critic | Critic Function | 轨迹评分项 |
| 非完整约束 | Nonholonomic Constraint | 如 DiffDrive $v_y = 0$ |
| Stateful Checker | — | XY 达标后锁定，只检航向 |

---

## 9.14 工程选型矩阵

| 场景 | 推荐控制器 | Goal Checker | 关键参数 |
|------|-----------|--------------|----------|
| 室内差速通用 | Graceful / RPP | SimpleGoalChecker | lookahead 0.4–0.6 m |
| 动态避障 | MPPI | SimpleGoalChecker | batch_size 2000 |
| 窄通道 | Graceful + collision | SimpleGoalChecker | use_collision_detection |
| 充电对接 | RPP / Graceful | StoppedGoalChecker | tolerance 0.05 m |
| 只到位置 | 任意 | PositionGoalChecker | — |
| 低算力嵌入式 | Pure Pursuit | SimpleGoalChecker | 10 Hz |
| 全向机器人 | MPPI (Omni) | SimpleGoalChecker | vy_std > 0 |
| 仿真调试 | Pure Pursuit | SimpleGoalChecker | OPEN_LOOP smoother |

**Autonomy 当前建议**：在控制器插件实现之前，可独立使用 Checker + VelocitySmoother 进行单元测试；FollowPath 循环完成后，优先集成 Graceful Controller 或 Regulated Pure Pursuit 作为首个可用插件。
