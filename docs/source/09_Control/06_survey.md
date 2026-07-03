# 6. 运动控制算法综述（Survey）

> **本文范围**：局部运动控制的**动机、历史、分类、算法谱系、工程选型**与 Autonomy 能力边界。  
> 形式化与流水线见 [§0 指南](00_guide.md#07-问题形式化)；架构见 [§2](02_architecture.md)；控制器索引见 [§5](05_controller_algorithms.md)（§5.2–§5.7 对应 `controller/10_*`–`15_*` 专题）。

---

## 6.1 综述定位

| 维度 | 本文 | 其他文档 |
|------|------|----------|
| $J(e,u)$ 形式化、FollowPath 时序 | 摘要 | [§0 指南](00_guide.md#07-问题形式化) · [§2 架构](02_architecture.md) |
| MPPI / RPP / DWB 公式与配置 | 摘要 + 链接 | [§10–§15](controller/10_graceful_controller.md) 专题 |
| 动机、发展史、分类、选型 | **本文** | — |

**建议阅读顺序**

| 角色 | 路径 |
|------|----------|
| 选型 / 集成 | §6.14 场景矩阵 → §6.15 决策树 → §6.10 排错 |
| 算法研发 | §6.2 动机 → §6.5 时间轴 → §6.6 分类 → §6.7 算法族 → 各 §10–§15 |
| 背景调研 | §6.5 → §6.13 业界生态 → §6.18 参考文献 |

---

## 6.2 为什么需要局部运动控制

### 6.2.1 全局规划 alone 不够

全局规划（Planning）在**静态或低频更新**的全局代价地图上输出几何路径 $\mathcal{P}$，通常假设：

- 机器人可瞬时达到路径切向速度（忽略动力学）
- 路径足够平滑、航点足够密
- 环境变化慢，重规划周期 1–5 Hz

实际运行中会出现：

|  gap | 后果 | 局部控制职责 |
|------|------|--------------|
| 动态障碍 | 全局路径被挡 | 局部 costmap + 避障 critic |
| 非完整约束 | DiffDrive 不能横移 | 运动学可行 $(v,\omega)$ |
| 动力学限制 | 加减速有上限 | VelocitySmoother / 控制器内约束 |
| 定位/地图误差 | 路径与真实环境偏差 | 10–50 Hz 闭环纠偏 |
| 路径离散 | 折线、急弯 | Lookahead / 优化带 |

因此导航栈经典分层为：**Planning 给「往哪走」**，**Control 给「现在怎么走」**。

### 6.2.2  deliberative vs reactive

| 范式 | 时间尺度 | 代表 | 在 Autonomy 中 |
|------|----------|------|----------------|
|  deliberative | 秒级 | 全局规划、行为树 | `planning` + `navigator` |
| reactive | 50–100 ms | DWA、MPPI、势场 | `control` |
|  hybrid | 双层 | Nav2 栈 | Planning + Control + 重规划 |

局部控制器本质是 **reactive policy** $\pi(x,\mathcal{P}, \mathcal{C}_{local}) \to u$，在控制周期内可重复计算，对传感器延迟更鲁棒。

### 6.2.3 与 Planning 的边界

```
Planning (1–5 Hz)          Control (10–50 Hz)
─────────────────          ────────────────────
全局几何路径 Path           速度 cmd_vel
静态/全局 costmap           局部 rolling costmap（可选）
无速度 profile              加速度/曲率约束
IsPathValid → 触发重规划    ProgressChecker → Recovery
```

二者通过 `planning_msgs::Path` 耦合；路径失效时 Navigator 重调 Planning，**不是** Control 的职责。

---

## 6.3 在导航栈中的位置

| 子问题 | 模块 | 频率 |
|--------|------|------|
| 定位 | `localization` | 10–50 Hz |
| 全局规划 | `planning` | 1–5 Hz |
| **局部控制** | **`control`** | **10–50 Hz** |
| 编排 | `navigator` | 事件驱动 |

```
Planning.Path ──→ ControllerServer ──→ cmd_vel ──→ 底盘
                      ↑        ↑
              local/global    odom + TF
                 costmap
                      ↓（可选）
              VelocitySmoother
```

Autonomy `control` 对齐 Nav2 `nav2_controller` + `nav2_velocity_smoother` + Goal/Progress Checker 体系。

---

## 6.4 问题形式化

### 6.4.1 状态与控制

差速/全向模型常用 $SE(2)$ 或扩展状态：

$$
x = (x, y, \theta, v_x, v_y, \omega_z)^\top, \quad
u = (v_x^{cmd}, v_y^{cmd}, \omega_z^{cmd})^\top
$$

给定参考路径 $\mathcal{P}=\{p_0,\ldots,p_N\}$，求 $\pi$ 使跟踪误差与控制代价最小：

$$
\min_{\pi} \int_0^T \Big( \|e_p\|_Q^2 + w_\theta e_\theta^2 + u^\top R u \Big) dt
$$

约束 $(x,y)\in\mathcal{C}_{free}$，$|v|\leq v_{max}$，$|a|\leq a_{max}$，DiffDrive 另有 $v_y=0$。

### 6.4.2 误差定义

**全局误差**：$e_p = [x-x_g,\, y-y_g]^\top$，$e_\theta = \mathrm{AngleDiff}(\theta,\theta_g)$。

**Frenet 误差**（沿路径）：$e_y$ 为有符号横向距离，$e_s$ 为弧长误差。几何控制器（PP、Stanley）多在 Frenet 或 lookahead 坐标下设计。

### 6.4.3 约束与能力矩阵

| 约束 | 说明 | 几何跟踪 | DWB/MPPI | TEB/MPC |
|------|------|----------|----------|---------|
| 几何避障 | footprint 不碰撞 | 弱/可选 | ✅ | ✅ |
| 非完整 | DiffDrive | ✅ | ✅ | ✅ |
| 运动学 | $R_{min}$ | RPP/MPPI | 部分 | ✅ |
| 动力学 | $a_{max}$ | Smoother | 仿真内 | ✅ |
| 时间最优 | $\min T$ | ❌ | 弱 | TEB ✅ |

---

## 6.5 发展时间轴

按五个阶段梳理；对照表见 [§6.5.1](#651-分阶段特征表)。

<div class="planning-timeline-v2">

<div class="timeline-era-block era-foundation">
  <div class="timeline-era-header">几何跟踪奠基 · 1980s–1990s</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1985</div>
      <div class="timeline-milestone-title">Pure Pursuit</div>
      <div class="timeline-milestone-desc">CMU 农业机器人；lookahead 曲率跟踪</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1990</div>
      <div class="timeline-milestone-title">Feedback Linearization</div>
      <div class="timeline-milestone-desc">非完整系统反馈线性化跟踪</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1995</div>
      <div class="timeline-milestone-title">Dynamic Window (DWA)</div>
      <div class="timeline-milestone-desc">Fox et al. 速度空间实时避障</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1995</div>
      <div class="timeline-milestone-title">Stanley</div>
      <div class="timeline-milestone-desc">DARPA Grand Challenge 横向控制</div>
    </div>
  </div>
</div>

<div class="timeline-era-block era-reactive">
  <div class="timeline-era-header">ROS 工程化 · 2000s–2010s</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2000s</div>
      <div class="timeline-milestone-title">move_base</div>
      <div class="timeline-milestone-desc">global_plan + base_local_planner (DWA/Trajectory Rollout)</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2009</div>
      <div class="timeline-milestone-title">TEB</div>
      <div class="timeline-milestone-desc">弹性带时空优化；动态障碍</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2010</div>
      <div class="timeline-milestone-title">DWB</div>
      <div class="timeline-milestone-desc">Critic 插件化 DWA；ROS navigation 标准</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2014</div>
      <div class="timeline-milestone-title">MPC 普及</div>
      <div class="timeline-milestone-desc">嵌入式 QP/SQP 求解器进入移动机器人</div>
    </div>
  </div>
</div>

<div class="timeline-era-block era-sampling">
  <div class="timeline-era-header">Nav2 时代 · 2018–2022</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2018</div>
      <div class="timeline-milestone-title">Navigation2</div>
      <div class="timeline-milestone-desc">Behavior Tree + 插件化 controller</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2019</div>
      <div class="timeline-milestone-title">Regulated Pure Pursuit</div>
      <div class="timeline-milestone-desc">Nav2 默认；曲率/障碍/目标速度调节</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2017–18</div>
      <div class="timeline-milestone-title">MPPI / Path Integral MPC</div>
      <div class="timeline-milestone-desc">Williams et al. 信息论随机 MPC</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2022</div>
      <div class="timeline-milestone-title">nav2_mppi_controller</div>
      <div class="timeline-milestone-desc">Nav2 官方 MPPI + Critics 生态</div>
    </div>
  </div>
</div>

<div class="timeline-era-block era-engineering">
  <div class="timeline-era-header">平滑与全栈整合 · 2023–至今</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2023</div>
      <div class="timeline-milestone-title">Graceful Controller</div>
      <div class="timeline-milestone-desc">Nav2 平滑跟踪；Park & Kuipers 2011 工程化</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2023</div>
      <div class="timeline-milestone-title">Velocity Smoother 独立节点</div>
      <div class="timeline-milestone-desc">Nav2 加速度约束后处理</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2024+</div>
      <div class="timeline-milestone-title">Learning / Diffusion Policy</div>
      <div class="timeline-milestone-desc">数据驱动局部策略（研究→产品探索）</div>
    </div>
    <div class="timeline-milestone timeline-milestone-highlight">
      <div class="timeline-milestone-year">2025</div>
      <div class="timeline-milestone-title">Autonomy control</div>
      <div class="timeline-milestone-desc">Nav2 架构移植；Checker/Smoother 就绪，插件待接</div>
    </div>
  </div>
</div>

</div>

### 6.5.1 分阶段特征表

| 阶段 | 年代 | 代表方法 | 核心突破 | 局限 |
|------|------|----------|----------|------|
| 几何跟踪 | 1985–1995 | PP, Stanley | 极简、低延迟、可证明局部稳定 | 不避障、切弯 |
| 反应式采样 | 1995–2010 | DWA, DWB | 实时局部避障、critic 可组合 | 局部最优、参数多 |
| 时空优化 | 2009–2018 | TEB, MPC | 动态障碍、约束显式 | 计算量、调参 |
| 随机 MPC | 2017–2022 | MPPI | 非线性+约束统一框架 | batch 算力需求 |
| 工程栈 | 2018–至今 | Nav2 插件族 | 产品级组合 | FollowPath+插件矩阵复杂 |
| 学习增强 | 2020–至今 | RL, Diffusion | 大数据泛化 | 安全可解释性 |

### 6.5.2 Autonomy 在技术谱系中的位置

```
1995 DWA → 2010 DWB → 2018 nav2_dwb → 2022 nav2_mppi / nav2_regulated_pp
                                              │
                                        2025 Autonomy control
                                        ├── Checker ×5 ✅
                                        ├── VelocitySmoother ✅
                                        ├── ControllerServer 骨架 ⏳
                                        └── 插件 RPP/Graceful/MPPI ⏳
```

Autonomy 定位于 **nav2 兼容的局部控制层**：接口与配置对齐，优先落地 RPP/Graceful，MPPI 为可选重算力方案。

---

## 6.6 算法分类体系

### 6.6.1 四维分类

```
局部运动控制
├── 按方法
│   ├── 几何：PP, Stanley, Graceful, RPP
│   ├── 采样：DWA, DWB, MPPI
│   ├── 优化：TEB, CHOMP, NMPC
│   └── 学习：RL, imitation, diffusion
├── 按感知
│   ├── 纯跟踪（仅路径+odom）
│   ├── 障碍感知（local costmap）
│   └── 预测性（+ prediction 模块）
├── 按模型
│   ├── 差速 SE(2)
│   ├── 全向 SE(2)+vy
│   └── Ackermann / 履带
└── 按最优性
    ├── 启发式（PP, DWB）
    ├── 滚动最优（MPC, MPPI）
    └── 局部最优（TEB）
```

### 6.6.2 方法—特性矩阵

| 方法族 | 代表 | 避障 | 路径质量 | 实时性 | 调参 | Autonomy |
|--------|------|------|----------|--------|------|----------|
| 几何跟踪 | PP, RPP, Graceful | 弱–中 | 中–高 | ★★★★★ | 低 | ⏳ RPP/Graceful |
| 速度采样 | DWB | 强 | 中 | ★★★★ | 高 | ❌ |
| 随机 MPC | MPPI | 强 | 高 | ★★★ | 高 | ⏳ 配置 |
| 时空优化 | TEB | 强 | 高 | ★★★ | 高 | ❌ |
| 经典 NMPC | acados, CasADi | 强* | 高 | ★★★ | 很高 | ❌ |
| 学习 | End-to-end | 变化 | 变化 | 推理快 | 数据 | ❌ |

\* 依赖模型与约束建模质量。

### 6.6.3 按感知依赖

| 类型 | costmap | odom | 预测 | 代表 |
|------|---------|------|------|------|
| 纯跟踪 | ❌ | ✅ | ❌ | Pure Pursuit |
| 障碍感知 | ✅ | ✅ | ❌ | DWB, MPPI |
| 预测避障 | ✅ | ✅ | ✅ | TEB + prediction |

---

## 6.7 核心算法详解

各算法**动机 → 核心公式 → 优劣 → Nav2/Autonomy 状态**。完整推导见 §10–§15 专题。

### 6.7.1 Pure Pursuit（1985）

**动机**：农业/户外低速场景，只需沿路径走，无需显式避障（障碍由全局规划处理）。

在路径上取 lookahead 距离 $L_d$ 处目标点，几何曲率：

$$
\kappa = \frac{2 y^*}{L_d^2}, \quad \omega = \kappa v
$$

| 优点 | 缺点 |
|------|------|
| $O(n)$ 极低延迟 | 不避障 |
| 工业验证数十年 | 急弯切内道 |
| 参数仅 $L_d$ | 无速度规划 |

Autonomy：`controller_utils` 中 `GetLookAheadPoint` 等已就绪，作为 RPP 核心。

### 6.7.2 Regulated Pure Pursuit（Nav2 2019）

**动机**：PP 在急弯、近障碍、近目标时速度过大 → 在 PP 上叠加**曲率调节、障碍减速、接近目标减速**。

$$
v_{cmd} = v_{pref} \cdot f_{curv}(\kappa) \cdot f_{obs}(d_{obs}) \cdot f_{goal}(d_{goal})
$$

Nav2 默认 indoor 控制器。Autonomy Phase 1 推荐首个插件（工具函数已有）。

详见 [§12 RPP](controller/12_rpp_controller.md)。

### 6.7.3 Stanley（2005 DARPA）

**动机**：高速自动驾驶需同时压横向与航向误差。

$$
\delta = e_\theta + \arctan\!\left(\frac{k_e e_y}{v + \varepsilon}\right)
$$

室内 AGV 较少用；Autonomy 未规划移植。

### 6.7.4 DWA / DWB（1995 / 2010）

**动机**：全局路径可能被**新出现的局部障碍**阻断，需在 $(v,\omega)$ 空间快速搜索可行速度。

**DWA**：动态窗口 $\mathcal{V}_d$ = 可达速度 ∩ 安全速度，评分 $G=\sum w_i S_i$。

**DWB**：Nav2 将 DWA **插件化**——Trajectory Generator + 多个 Critic（PathDist, GoalDist, ObstacleFootprint, RotateToGoal…）。

| 优点 | 缺点 |
|------|------|
| 成熟、生态丰富 | critic 权重调参复杂 |
| 实时局部避障 | 高维 (vx,vy,ω) 采样爆炸 |
| 可组合行为 | 轨迹前向仿真近似 |

Autonomy：未实现；动态避障需求高时可评估移植或直接用 MPPI。

详见 [§13 DWB](controller/13_dwb_controller.md)。

### 6.7.5 MPPI（2017–2022）

**动机**：统一处理非线性动力学、多目标代价、约束，避免手工设计 DWB critic 权重；利用 GPU/多核并行采样。

路径积分近似：

$$
u^* \approx \sum_{k=1}^{K} w_k u^{(k)}, \quad
w_k = \frac{\exp(-S(u^{(k)})/\lambda)}{\sum_j \exp(-S(u^{(j)})/\lambda)}
$$

Nav2 `nav2_mppi_controller`：$K \sim 2000$，$H \sim 56$，多 CostCritic（Obstacle, PathFollow, Goal, PreferForward…）。

| 优点 | 缺点 |
|------|------|
| 非线性+约束自然融合 | 算力需求大 |
| Critics 可扩展 | temperature、noise_std 敏感 |
| 支持 Omni/Ackermann | 需与 control_frequency 对齐 |

Autonomy：`controller.lua` 已含完整 MPPI 配置；插件待实现。

详见 [§11 MPPI](controller/11_mppi_controller.md)。

### 6.7.6 TEB（2009–2017）

**动机**：DWB 轨迹无显式**时间参数**，难以协调动态障碍与速度；TEB 同时优化 pose 序列与时间间隔 $\Delta t_i$。

$$
\min \sum_i \Big( w_t \Delta t_i^2 + w_o d_{obs,i}^2 + w_v (v_i - v_{pref})^2 + w_a a_i^2 \Big)
$$

适合人群、叉车等动态场景；可接 [Prediction](../11_Prediction/08_behavior_prediction.md) 模块。Autonomy 暂未集成。

详见 [§14 TEB](controller/14_teb_controller.md)。

### 6.7.7 Graceful Controller（2023）

**动机**：室内服务机器人要**平滑**到达（初始对齐、减速区、少倒车），比 RPP 更强调 UX。

基于 Park & Kuipers (ICRA 2011) 平滑控制律 + Nav2 工程封装（slowdown_radius、collision check）。

Autonomy 配置已预留，Phase 2 推荐。

详见 [§10 Graceful](controller/10_graceful_controller.md)。

### 6.7.8 经典 NMPC（与 MPPI 区分）

**动机**：显式求解有限时域 OCP，硬约束保证性更好（泊车、高速）。

$$
\min_{\mathbf{U}} \sum_{k=0}^{H-1} \ell(x_k,u_k) + \ell_f(x_H) \quad \text{s.t. 动力学与约束}
$$

求解：SQP / RTI / acados。与 MPPI（采样+softmax）互补。Autonomy 未实现，见 [§15 MPC](controller/15_mpc_controller.md)。

### 6.7.9 Learning-based 局部控制

| 路线 | 动机 | 现状 |
|------|------|------|
| 端到端 | 省掉手工 critic | 仿真→真机 gap、安全 |
| RL local planner | 自适应复杂环境 | 训练成本高 |
| Diffusion policy | 多模态轨迹 | 2024+ 研究热点 |

工程产品仍以 Nav2 经典控制器为主；学习方案多作辅助（启发式、perception-to-cost）。

---

## 6.8 辅助组件

局部控制不仅是「算 $(v,\omega)$」，Nav2 将**到达判定、卡住检测、速度后处理**独立为可组合组件。

### 6.8.1 Goal Checker

| 类型 | 判定 | 典型场景 |
|------|------|----------|
| SimpleGoalChecker | XY + yaw | 默认导航 |
| PositionGoalChecker | 仅 XY | 不要求最终朝向 |
| StoppedGoalChecker | XY + yaw + $\|v\|<\epsilon$ | 充电对接 |

Autonomy ✅ 三种均已实现。见 [§3 Checkers](03_checkers.md)。

### 6.8.2 Progress Checker

检测机器人是否在**时间窗口内**有足够位移/转角，否则 Navigator 触发 Recovery。

Autonomy ✅ Simple / Pose 两种；时间窗口待完善。

### 6.8.3 VelocitySmoother

**动机**：控制器输出可能阶跃，底盘驱动器/电机有 $a_{max}$ 与 deadband。

约束 $\Delta v$  per cycle，支持 OPEN/CLOSED_LOOP。Autonomy 算法 ✅，节点接线 ⏳。

见 [§4 VelocitySmoother](04_velocity_smoother.md)。

---

## 6.9 Autonomy control 能力边界

### 6.9.1 已实现

| 组件 | 状态 |
|------|------|
| `ControllerInterface` 插件接口 | ✅ |
| Goal Checker ×3 | ✅ |
| Progress Checker ×2 | ✅ |
| VelocitySmoother / OdomSmoother | ✅ |
| 几何工具（lookahead、圆-线段交点） | ✅ |
| Lua → `ControllerOptions` | ✅ |
| `ControllerServer` 骨架 | ⏳ 部分 |

### 6.9.2 待实现（优先级）

| 组件 | 优先级 | 说明 |
|------|--------|------|
| FollowPath 控制循环 | P0 | `ComputeControl()` 等 |
| Autolink 插件加载 | P0 | 参照 `PlannerServer` |
| RPP / Graceful 插件 | P1 | 首个可用控制器 |
| CheckerOptions Lua 接线 | P1 | proto 已有 |
| MPPI 插件 | P2 | 可选编译 |
| VelocitySmoother 节点 | P2 | pub/sub |

### 6.9.3 与 Nav2 能力矩阵

| 能力 | Nav2 | Autonomy |
|------|------|----------|
| FollowPath | ✅ | ⏳ |
| Regulated PP | ✅ | ❌ |
| Graceful | ✅ | ⏳ 配置 |
| MPPI | ✅ | ⏳ 配置 |
| DWB | ✅ | ❌ |
| TEB | ✅ | ❌ |
| Goal/Progress Checker | ✅ | ✅ |
| Velocity Smoother | ✅ | ✅ 未接线 |

---

## 6.10 工程质量与影响因素

### 6.10.1 控制频率

| 频率 | 适用 | 说明 |
|------|------|------|
| 10 Hz | 低算力 / 慢速 AGV | 延迟明显 |
| **20 Hz** | **默认推荐** | 与 MPPI `dt=0.05` 一致 |
| 50 Hz | 高速 / 全向 | 需更强 CPU/GPU |

### 6.10.2 Lookahead 与容差

| 参数 | 小 | 中 | 大 |
|------|----|----|-----|
| $L_d$ | 跟踪紧、易抖 | 0.4–0.6 m 常用 | 切弯、平滑 |
| `xy_goal_tolerance` | 精度高 | 0.25 m | 易报到达 |
| `required_movement_radius` | 易触发 recovery | 0.5 m | 卡住不敏感 |

### 6.10.3 鱼骨图：跟踪质量不佳

<div class="fishbone-wrap">

<div class="fishbone-label">上层原因</div>
<div class="fishbone-grid">
  <div class="fishbone-card">
    <h4>路径与规划</h4>
    <ul>
      <li>航点过疏/过密</li>
      <li>全局路径急弯</li>
      <li>重规划滞后</li>
      <li>Path 坐标系错误</li>
    </ul>
  </div>
  <div class="fishbone-card">
    <h4>控制器与参数</h4>
    <ul>
      <li>控制器选型不当</li>
      <li>lookahead 不匹配速度</li>
      <li>MPPI critic 权重失衡</li>
      <li>max_vel 与地图不匹配</li>
    </ul>
  </div>
  <div class="fishbone-card">
    <h4>机器人模型</h4>
    <ul>
      <li>footprint 偏差</li>
      <li>非完整未建模</li>
      <li>滑移/打滑</li>
    </ul>
  </div>
</div>

<div class="fishbone-head">局部跟踪质量不佳</div>

<div class="fishbone-label">下层原因</div>
<div class="fishbone-grid">
  <div class="fishbone-card fishbone-lower">
    <h4>感知与地图</h4>
    <ul>
      <li>local costmap 延迟</li>
      <li>膨胀不足</li>
      <li>动态障碍未入图</li>
    </ul>
  </div>
  <div class="fishbone-card fishbone-lower">
    <h4>定位与 TF</h4>
    <ul>
      <li>odom 漂移</li>
      <li>map→odom 跳变</li>
      <li>控制频率 &lt; 定位频率</li>
    </ul>
  </div>
  <div class="fishbone-card fishbone-lower">
    <h4>执行与平滑</h4>
    <ul>
      <li>VelocitySmoother 过限</li>
      <li>deadband 过大</li>
      <li>底盘 PID 饱和</li>
    </ul>
  </div>
</div>

</div>

### 6.10.4 因素—对策速查

| 分支 | 症状 | 对策 |
|------|------|------|
| 路径 | 切弯、振荡 | 增大 $L_d$；Planning 换 Theta* |
| 控制器 | 贴障、卡住 | 换 MPPI/DWB；调 ObstacleCritic |
| 定位 | 路径相对漂移 | 查 TF；提高 control 频率 |
| 平滑 | 起步慢、到不了 | 调 `max_accel`；CLOSED_LOOP |
| Checker | 早报到达 | 收紧 tolerance 或 StoppedGoalChecker |

更多见 [§0.19 排错](00_guide.md#019-故障排查)。

---

## 6.11 算法复杂度对比

| 算法 | 复杂度 | 20 Hz 典型耗时 | 内存 |
|------|--------|----------------|------|
| Pure Pursuit | $O(n)$ | &lt; 0.1 ms | 低 |
| Regulated PP | $O(n)$ | &lt; 0.5 ms | 低 |
| Graceful | $O(n)$ | &lt; 1 ms | 低 |
| DWB | $O(n_{traj}\cdot n_{sample})$ | 1–5 ms | 中 |
| MPPI ($2000\times56$) | $O(K\cdot H)$ | 5–20 ms | 高 |
| TEB | $O(n^2)$ iter | 10–50 ms | 中 |
| NMPC | $O(n_x^3 H)$ | 5–30 ms | 中–高 |

\* 随 CPU/GPU、costmap 大小变化。

---

## 6.12 与其他模块的耦合

```
planning ──Path──► controller_server ──cmd_vel──► 底盘
    │                    ↑    ↑
global costmap      local costmap
    │                    ↑
   map ◄──scan── perception
    │
localization ──odom/TF──► controller
    │
navigator ──FollowPath──► controller
         ◄──goal/progress check──┘
```

| 模块 | 数据 | 说明 |
|------|------|------|
| `planning` | `Path` | 全局参考 |
| `map` | global/local costmap | MPPI/DWB 避障 |
| `localization` | odom, TF | 闭环 |
| `navigator` | FollowPath, recovery | 编排 |
| `prediction` | 动态障碍（可选） | TEB/MPPI 扩展 |

---

## 6.13 业界生态

### 6.13.1 Nav2 控制器生态

| 控制器 | 包 | 算法 | Autonomy |
|--------|-----|------|----------|
| Regulated PP | nav2_regulated_pure_pursuit | 几何+调节 | ⏳ |
| Graceful | nav2_graceful_controller | 平滑跟踪 | ⏳ 配置 |
| MPPI | nav2_mppi_controller | 随机 MPC | ⏳ 配置 |
| DWB | nav2_dwb_controller | DWA+critics | ❌ |
| Rotation Shim | nav2_rotation_shim | 对齐辅助 | ❌ |
| TEB | teb_local_planner (3rd party) | 弹性带 | ❌ |

### 6.13.2 其他框架

| 框架 | 局部控制 | 特点 |
|------|----------|------|
| Navigation2 | 插件多种 | ROS 2 标准 |
| Autonomy | 对齐 Nav2 | autolink |
| ROS 1 move_base | DWA/TEB | legacy |
| Apollo | MPC + 规划 | 自动驾驶 |
| Isaac Sim / Nav2 | MPPI GPU | 仿真验证 |

---

## 6.14 工程选型矩阵

| 场景 | 推荐控制器 | Goal Checker | 关键参数 |
|------|-----------|--------------|----------|
| 室内差速通用 | Graceful / RPP | SimpleGoalChecker | lookahead 0.4–0.6 m |
| 动态避障 | MPPI | SimpleGoalChecker | batch_size 2000 |
| 窄通道 | Graceful + collision | SimpleGoalChecker | `use_collision_detection` |
| 充电对接 | RPP / Graceful | StoppedGoalChecker | tolerance 0.05 m |
| 只到位置 | 任意 | PositionGoalChecker | — |
| 低算力嵌入式 | RPP / PP | SimpleGoalChecker | 10 Hz |
| 全向机器人 | MPPI (Omni) | SimpleGoalChecker | `vy_std > 0` |
| 仿真调试 | RPP | SimpleGoalChecker | OPEN_LOOP smoother |
| 高速户外 | Stanley / MPC | StoppedGoalChecker | 模型精确 |
| 人群密集 | MPPI / TEB | SimpleGoalChecker | 高 obstacle critic |

**Autonomy 当前建议**：FollowPath 循环完成后，**Phase 1** 上 RPP 验证闭环；**Phase 2** Graceful；**Phase 3** MPPI。Checker + Smoother 可先行单元测试。

---

## 6.15 选型决策树

```
需要沿 Path 跟踪并输出 cmd_vel？
├── 否 → 非 control 职责（behavior / manipulation）
└── 是
    ├── 环境动态障碍多？
    │   ├── 是 → MPPI（算力够）或 DWB/TEB
    │   └── 否
    │       ├── 强调平滑 UX？ → Graceful
    │       ├── 要低延迟/嵌入式？ → RPP / Pure Pursuit
    │       └── 默认 → RPP（Nav2 默认等价）
    ├── 全向底盘？ → MPPI Omni 模型
    ├── 充电/精密停靠？ → StoppedGoalChecker
    └── 算力/频率
        ├── MPPI 需 GPU 或多核 + ≥20 Hz
        └── RPP/Graceful 10–20 Hz 即可
```

---

## 6.16 未来趋势与路线图

<div class="roadmap-steps">
  <div class="roadmap-step roadmap-current">
    <strong>2025</strong>
    <span>Autonomy<br/>Checker+Smoother 就绪</span>
  </div>
  <div class="roadmap-step">
    <strong>2026</strong>
    <span>RPP / Graceful<br/>首个插件</span>
  </div>
  <div class="roadmap-step">
    <strong>2027</strong>
    <span>MPPI 可选<br/>GPU 加速</span>
  </div>
  <div class="roadmap-step">
    <strong>2028</strong>
    <span>Prediction<br/>+ MPPI/TEB</span>
  </div>
  <div class="roadmap-step">
    <strong>2029+</strong>
    <span>学习辅助<br/>critic / policy</span>
  </div>
</div>

| 方向 | 说明 |
|------|------|
| GPU MPPI | batch 10k+，Isaac/Nav2 验证 |
| 规划-控制联合 | 单 OCP 替代分层 |
| Semantic control | 语义地图调速（人行道/坡道） |
| 多机协调 | 速度障碍、MAPF 与 local control 结合 |
| Safe RL | 约束 RL 局部策略 |

---

## 6.17 术语表

| 术语 | 英文 | 说明 |
|------|------|------|
| 局部规划 | Local Planning | 与 Control 同义（输出 cmd_vel） |
| 轨迹跟踪 | Trajectory Tracking | 沿 Path 运动 |
| Lookahead | Look-ahead Distance | 前瞻 $L_d$ |
| 动态窗口 | Dynamic Window | 可达速度集 $\mathcal{V}_d$ |
| Critic | Critic Function | DWB/MPPI 评分项 |
| 非完整约束 | Nonholonomic | DiffDrive $v_y=0$ |
| Stateful Checker | — | XY 达标后只检 yaw |
| Path Integral MPC | MPPI | 采样型随机 MPC |

---

## 6.18 参考文献

**教材**

1. [Siegwart et al., *Autonomous Mobile Robots* (2011)](https://mitpress.mit.edu/9780262015356/autonomous-mobile-robots/)
2. [Rawlings & Mayne, *Model Predictive Control* (2009)](https://sites.engineering.ucsb.edu/~jbraw/mpc/)

**里程碑论文**

| 年份 | 文献 | 贡献 |
|------|------|------|
| 1992 | [Coulter, Pure Pursuit (CMU-RI-TR-92-01)](https://www.ri.cmu.edu/publications/implementation-of-the-pure-pursuit-path-tracking-algorithm/) | PP |
| 1997 | [Fox et al., DWA (IEEE RAM)](https://doi.org/10.1109/100.580977) | 动态窗口 |
| 2006 | [Thrun et al., Stanley (JFR)](https://doi.org/10.1002/rob.20147) | 横向控制 |
| 2011 | [Park & Kuipers, Smooth Control (ICRA)](https://doi.org/10.1109/ICRA.2011.5980227) | Graceful 理论基础 |
| 2017 | [Rösmann et al., TEB (IEEE RAM)](https://doi.org/10.1109/MRA.2016.2582927) | 弹性带 |
| 2017 | [Williams et al., IT-MPC (ICRA)](https://ieeexplore.ieee.org/document/7989202) | MPPI |
| 2022 | [Macenski et al., Nav2 (Science Robotics)](https://www.science.org/doi/10.1126/scirobotics.abm6074) | 工程栈 |

**工程**

- [Navigation2 Controllers](https://docs.nav2.org/configuration/packages/configuring-controller-server.html)
- [nav2_mppi_controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller)
- [nav2_graceful_controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_graceful_controller)

---

## 6.19 相关文档

- [§0 指南](00_guide.md) · [§2 架构](02_architecture.md)
- [§3 Checkers](03_checkers.md) · [§4 Smoother](04_velocity_smoother.md) · [§5 控制器总览](05_controller_algorithms.md)
- [Planning 综述](../08_Planning/06_survey.md) · [Navigator 综述](../16_Navigator/09_survey.md)
