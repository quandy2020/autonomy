# 6. 轨迹规划综述（Survey）

> **本文范围**：局部**轨迹规划**与运动控制的**动机、历史、分类、算法谱系**（含**时空联合**优化）、工程选型与 Autonomy 能力边界。  
> 形式化与流水线见 [§0 指南](00_guide.md#07-问题形式化)；架构见 [§2](02_architecture.md)；控制器索引见 [§5](05_controller_algorithms.md)（§5.2–§5.7 对应 `controller/10_*`–`15_*` 专题）。

---

## 6.1 综述定位

| 维度 | 本文 | 其他文档 |
|------|------|----------|
| $J(e,u)$ 形式化、FollowPath 时序 | 摘要 | [§0 指南](00_guide.md#07-问题形式化) · [§2 架构](02_architecture.md) |
| MPPI / RPP / DWB / TEB 公式 | 摘要 + 链接 | [§10–§15](controller/10_graceful_controller.md) 专题 |
| 几何路径 vs 轨迹、时空联合谱系 | **本文 §6.2.4–§6.6.5** | [Planning 综述](../08_Planning/06_survey.md)（全局几何 · [§6.5.4](../08_Planning/06_survey.md#654-采样优化与反应式未内置)） |
| 动机、发展史、分类、选型 | **本文** | — |

**建议阅读顺序**

| 角色 | 路径 |
|------|----------|
| 选型 / 集成 | §6.4.4 路径/轨迹 → §6.6.4–§6.6.5 时空联合 → [§0.9.1](00_guide.md#091-局部轨迹与时空联合选型) / [§5.8.1](05_controller_algorithms.md#581-控制器与-checker-配对) → §6.15 决策树 |
| 算法研发 | §6.2 → §6.4.4 → §6.6.4–§6.6.5 → §6.7 → 各 §10–§15 |
| 背景调研 | §6.5 时间轴 → §6.6.4–§6.6.5 → §6.7.13 预测时空 → §6.13 → §6.18 参考文献 |

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

### 6.2.4 几何路径、轨迹与局部轨迹规划

导航文献中常混用 *path* 与 *trajectory*；在 Autonomy 分层下建议区分：

| 对象 | 符号 / 表示 | 是否含时间 | 典型产出模块 |
|------|-------------|------------|--------------|
| **几何路径** | $\mathcal{P}=\{p_i\}\subset SE(2)$ | ❌ | `planning`（NavFn / Theta*） |
| **速度剖面** | $(v(s),\,\omega(s))$ 或 $v(t)$ | 部分 | time-scaling、Smoother |
| **轨迹** | $\tau(t)=(x,y,\theta)(t)$ 或 $(x,y,\theta,v,\omega)(t)$ | ✅ | 控制器 rollout / 优化输出 |
| **时空轨迹** | $\{s_k,\,\Delta t_k\}$ 或 $B$（TEB） | ✅ 显式 | TEB、NMPC、CHOMP |

**局部轨迹规划**（本文 Control 层）在几何路径之上补全**可行运动**与（可选）**时间分配**：

```
Planning.Path  ──►  [纯跟踪] PP/RPP/Graceful     ──► cmd_vel(t)
                 ──►  [采样] DWB/MPPI rollout   ──► 隐式 τ(t)
                 ──►  [时空联合] TEB/NMPC/CHOMP ──► τ*(t) + u*(t)
```

- **纯几何跟踪**：只优化当前 $(v,\omega)$，不显式存储整条 $\tau(t)$（RPP、Graceful）。
- **滚动 rollout**：在预测时域内生成离散 $\tau_{0:H}$，执行首步（DWB、MPPI）。
- **时空联合**：决策变量同时含**空间形变**与**时间参数**（或控制序列 $\mathbf{U}_{0:H-1}$），适合动态障碍、加速度/jerk 约束。详见 [§6.6.4](#664-时空联合轨迹规划)。

全局 **SE(2) 路径搜索**（栅格 A*、Hybrid A*）见 [Planning 综述 §6.3](../08_Planning/06_survey.md#631-四维分类法)；本文聚焦**局部**与 **kinodynamic / 时空** 族。

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

### 6.4.4 轨迹优化问题（局部）

给定初值路径或 pose 序列，局部轨迹规划常写为：

$$
\min_{\tau,\,\mathbf{U}} \;
\sum_{k=0}^{H-1} \ell\bigl(x_k,u_k\bigr) + \ell_f(x_H)
\quad \text{s.t.}\quad
x_{k+1}=f(x_k,u_k),\;
(x_k,u_k)\in\mathcal{F}
$$

- **$\ell$**：跟踪 $\mathcal{P}$、避障、控制 effort、**时间**（$\Delta t_k$ 或 $\|u\|$）。
- **$\mathcal{F}$**：非完整、$v,a,\omega$ 界、碰撞约束（硬约束或罚函数）。
- **与 MPPI**：同一 OCP 结构；MPPI 用路径积分采样近似 $\arg\min$，NMPC/TEB 用 NLP/SQP/LM 显式求解（[§15 MPC](controller/15_mpc_controller.md)）。

**Time-scaling**（仅对几何路径补时间，不做空间 deform）：$s(t)$ 单调，$\dot{s}$ 受 $v_{max}(s)$、$\kappa(s)$ 限制（Bang-coast-bang、S 曲线）。RPP/Graceful 的减速区属于**启发式 time-scaling**，非全局时间最优。

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
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2013</div>
      <div class="timeline-milestone-title">CHOMP</div>
      <div class="timeline-milestone-desc">轨迹优化协变梯度；障碍势场 + 平滑</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2014</div>
      <div class="timeline-milestone-title">TrajOpt / STOMP</div>
      <div class="timeline-milestone-desc">序列凸优化 · 随机轨迹优化并行</div>
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
| 几何跟踪 | 1985–1995 | PP, Stanley | 极简、低延迟、可证明局部稳定 | 不避障、无时间剖面 |
| 反应式采样 | 1995–2010 | DWA, DWB | 实时局部避障、critic 可组合 | 局部最优、单步/短 horizon |
| 轨迹优化 | 2010–2018 | CHOMP, TrajOpt, STOMP | 空间轨迹平滑 + 障碍代价 | 局部最优、初值敏感 |
| 时空联合 | 2009–2018 | TEB, timed-elastic, NMPC | **位姿 + $\Delta t$** 联合优化 | 计算量、非凸 |
| 随机 MPC | 2017–2022 | MPPI | 非线性+约束统一框架 | batch 算力需求 |
| 工程栈 | 2018–至今 | Nav2 插件族 | 产品级组合 | 插件矩阵复杂 |
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
局部轨迹规划
├── 按方法
│   ├── 几何：PP, Stanley, Graceful, RPP
│   ├── 采样：DWA, DWB, MPPI
│   ├── 轨迹优化（空间）：CHOMP, TrajOpt, STOMP
│   ├── 时空联合：TEB, EB, NMPC, kinodynamic RRT*
│   └── 学习：RL, imitation, diffusion
├── 按是否显式时间
│   ├── 无时间剖面：PP, RPP, Graceful
│   ├── 隐式（rollout 步长）：DWB, MPPI
│   └── 显式 $\Delta t$ / $H$：TEB, MPC, CHOMP 离散化
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
    └── 局部 NLP（TEB, TrajOpt）
```

### 6.6.2 方法—特性矩阵

| 方法族 | 代表 | 避障 | 路径质量 | 实时性 | 调参 | Autonomy |
|--------|------|------|----------|--------|------|----------|
| 几何跟踪 | PP, RPP, Graceful | 弱–中 | 中–高 | ★★★★★ | 低 | ⏳ RPP/Graceful |
| 速度采样 | DWB | 强 | 中 | ★★★★ | 高 | ❌ |
| 随机 MPC | MPPI | 强 | 高 | ★★★ | 高 | ⏳ 配置 |
| 轨迹优化 | CHOMP, TrajOpt | 强 | 高 | ★★★ | 高 | ❌ |
| 时空联合 | TEB | 强 | 高 | ★★★ | 高 | ❌ |
| 经典 NMPC | acados, CasADi | 强* | 高 | ★★★ | 很高 | ❌ |
| 学习 | End-to-end | 变化 | 变化 | 推理快 | 数据 | ❌ |

\* 依赖模型与约束建模质量。

### 6.6.3 按感知依赖

| 类型 | costmap | odom | 预测 | 代表 |
|------|---------|------|------|------|
| 纯跟踪 | ❌ | ✅ | ❌ | Pure Pursuit |
| 障碍感知 | ✅ | ✅ | ❌ | DWB, MPPI |
| 预测避障 | ✅ | ✅ | ✅ | TEB + prediction |

### 6.6.4 时空联合轨迹规划

**动机**：仅跟踪几何路径无法协调**何时**到达路径各点；动态障碍需要预测时间维上的冲突。时空联合方法在优化变量中同时包含**空间配置**与**时间参数**（或等价地，固定 $\Delta t$ 下的控制序列 $\mathbf{U}$）。

| 族 | 决策变量 | 时间如何进入 | 代表 | Nav2 / Autonomy |
|----|----------|--------------|------|-----------------|
| **Timed Elastic Band** | 位姿 $s_k$ + 段时长 $\Delta t_k$ | 显式 $\Delta t_k$；共弧 + 速度界 | TEB (Rösmann) | 第三方 `teb_local_planner` · ❌ |
| **Elastic Band（经典）** | 仅位姿链 | 事后 time-scaling | EB, NavFn 局部 | — |
| **MPC / NMPC** | $\mathbf{U}_{0:H-1}$ 或 $x_k$ | 固定 $H\Delta t$ 网格 | acados, CasADi | ❌ · 见 [§15](controller/15_mpc_controller.md) |
| **MPPI** | 采样 $\mathbf{U}^{(k)}$ | rollout 步长 $\Delta t$ | nav2_mppi | ⏳ · [§11](controller/11_mppi_controller.md) |
| **CHOMP / TrajOpt** | 离散 $x_k$ 或 waypoints | 均匀 $\Delta t$ 或弧长参数 | MoveIt, 研究栈 | ❌ |
| **Kinodynamic 搜索** | 状态-时间 $(x,t)$ 图 | 边带 $\Delta t$ | Hybrid A* 局部、RRT* | Planning 层为主 |

**与分层栈的关系**：

```
                    ┌─ 仅空间 deform ─ CHOMP, TrajOpt
Planning.Path ──────┼─ 空间 + 显式 Δt ─ TEB
                    └─ 控制序列 u_k ─── MPC, MPPI, DWB
```

- **TEB** 是 Nav2 生态中最典型的**显式时空联合**局部规划器：$B=\{s_1,\Delta T_1,\ldots,s_n\}$（第三方 `teb_local_planner`，Autonomy 未集成）。
- **MPC/MPPI** 不显式优化 $\Delta t_k$，但 $H$ 与 $\Delta t$ 固定时，$\{x_k\}_{0:H}$ 仍是一条**离散时空轨迹**。
- **Planning** 输出的 Path **不含** $t$；时空联合发生在 **Control**（或 move_base 的 *local planner* 角色）。

**选型提示**：动态障碍多、需 jerk/加速度协调 → TEB / MPPI / NMPC；静态窄道、低算力 → RPP / Graceful（几何 + 启发式减速）。

### 6.6.5 Elastic Band 与时空参数化演进

局部「deform 路径 + 满足约束」的思想可沿**时间是否显式优化**分为三代：

| 阶段 | 表示 | 时间 | 代表 | 特点 |
|------|------|------|------|------|
| **Elastic Band (EB)** | 位姿链 $\{s_k\}$ | 事后 time-scaling | Metzig & Günther, EB 局部规划 | 仅空间 deform；$\Delta t$ 由 $v_{max}$ 导出 |
| **Timed EB / TEB** | $s_k$ 与 $\Delta T_k$ 交错 | **优化变量** | Rösmann et al. 2009–2017 | 同一 NLP 协调 kinodynamic + 动态障碍 |
| **控制序列 MPC** | $x_k,u_k$ 网格 | 固定 $H\Delta t$ | NMPC, MPPI, DWB | 不显式 $\Delta T_k$，但 $\{x_k\}$ 即离散时空轨迹 |

```
Planning.Path ──► EB：deform {s_k} ──► time-scale ──► cmd_vel
              ──► TEB：min V({s_k, ΔT_k})
              ──► MPC/MPPI：min Σ ℓ(x_k,u_k), x_{k+1}=f(x_k,u_k)
```

- **EB → TEB**：从「先形后时」到「形时同优化」，是 Nav2 生态选用 TEB 而非纯 EB 的主因。
- **TEB vs MPC**：TEB 稀疏 band + LM；MPC 固定 horizon 网格 + SQP/采样。动态障碍多时 TEB 可接 prediction 时间窗；MPC/MPPI 在 critic/约束中编码预测轨迹（见 [§6.7.13](#6713-预测感知时空规划)）。
- **Hybrid A***：在 **Planning** 层做 kinodynamic **全局**搜索 $(x,t)$；局部仍常用 TEB/MPPI 跟踪或短 horizon 重优化（[Planning §6.5.4](../08_Planning/06_survey.md#654-采样优化与反应式未内置)）。

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

**动机**：经典 Elastic Band 只 deform **位姿**，时间靠事后 scaling；DWB/MPPI 的 rollout **步长固定**但无显式 $\Delta t_k$ 优化。TEB 在 **Timed Elastic Band** 中联合优化 $s_k$ 与 $\Delta T_k$，将 kinodynamic 与避障写入同一稀疏 NLP（Rösmann et al., IROS 2017）。

$$
B = \{ s_1,\, \Delta T_1,\, s_2,\, \ldots,\, \Delta T_{n-1},\, s_n \}, \quad
\min_B \tilde{V}(B) = \sum_k \big( \phi_{\mathrm{obs},k} + \phi_{\mathrm{kin},k} + \chi_{\mathrm{time},k} + \ldots \big)
$$

| 优点 | 缺点 |
|------|------|
| **时空联合**，动态障碍可预测 | 非凸，LM 局部最优 |
| 显式 $v_k,a_k$ 由 $\Delta T_k$ 导出 | 参数/权重多 |
| car-like / 差速共框架 | Autonomy 未集成 |

适合人群、叉车等动态场景；可接 [Prediction](../11_Prediction/08_behavior_prediction.md) 模块。

详见 Rösmann et al. (IROS 2017; IEEE RAM 2017)。

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

### 6.7.10 CHOMP（2009–2013）

**动机**：在**固定时间网格**上对离散轨迹 $x_{0:H}$ 做协变梯度下降，用**障碍势场** + **平滑正则** deform 路径，无需手工 DWA 评分权重。

$$
\min_{x_{0:H}} \sum_k \Big( c_{\mathrm{obs}}(x_k) + \lambda \|\nabla x_k\|^2 + \mu \|\nabla^2 x_k\|^2 \Big)
$$

| 特点 | 说明 |
|------|------|
| 空间轨迹优化 | 时间通常均匀 $\Delta t$；**非**显式 $\Delta t_k$ 决策 |
| 局部方法 | 初值敏感，易陷局部极小 |
| 生态 | 源于 MoveIt! 机械臂；平面移动机器人有移植 |

Autonomy 未集成；与 TEB 相比缺少显式 timed band，与 MPPI 相比无约束硬保证。

**文献**：Ratliff et al., *CHOMP: Gradient Optimization Techniques for Efficient Motion Planning*, ICRA 2009；Zucker et al., 2013 协变版本。

### 6.7.11 TrajOpt / STOMP（2013–2014）

**TrajOpt**（Schulman et al.）：将碰撞等不等式约束序列凸化（SCS），在 waypoints 上迭代求解**局部轨迹**；适合低维 C-space，移动机器人多用于**离线**或短 horizon 重规划。

**STOMP**（Kalakrishnan et al.）：对轨迹加**随机扰动**，用加权更新（与 MPPI 同属随机优化族，但更新对象为**整条轨迹形状**而非控制序列）。

| 方法 | 优化对象 | 与 MPPI/TEB 关系 |
|------|----------|------------------|
| TrajOpt | waypoint / $x_k$ | 确定性 NLP；无 softmax |
| STOMP | 轨迹分布 | 随机采样轨迹；无 MPC 滚动 |
| MPPI | $\mathbf{U}_{0:H-1}$ | 滚动 + 路径积分 |
| TEB | $s_k,\,\Delta T_k$ | 显式时空联合 |

Autonomy 未集成；动态局部场景更常选 **MPPI**（Nav2 官方）或 **TEB**（第三方）。

### 6.7.12 Time-scaling 与 jerk 约束

**动机**：全局 Path 几何可行但**无速度剖面**；工业 AGV 需限制 $v,a,jerk$（Graceful / 轮椅场景，见 [§10](controller/10_graceful_controller.md)）。

常见两阶段：

1. **几何路径** $\mathcal{P}(s)$（Planning 或 Pure Pursuit 隐式）
2. **Time-scaling** $s(t)$：$\dot{s}\leq v_{max}(s)$，$\ddot{s}\leq a_{max}$，有时加 jerk 界

Park & Kuipers (2011) 在平滑控制律上通过 motion target 序列实现**有界 $v,a,jerk$**；Nav2 **VelocitySmoother** 在 $u$ 层做后处理（[§4](04_velocity_smoother.md)）。这与 TEB/NMPC 的**单阶段时空联合**互补：实现简单 vs 全局协调。

### 6.7.13 预测感知时空规划

**动机**：静态 costmap 假设障碍不动；行人、叉车需 **$(x,y,t)$ 冲突检测**。Prediction 模块输出障碍未来轨迹 $\hat{o}_j(t)$，局部规划在时空上留 clearance。

| 集成方式 | 做法 | 代表 |
|----------|------|------|
| **TEB 动态项** | $\phi_{\mathrm{dyn},k}(s_k,t_k)$ 惩罚与 $\hat{o}(t)$ 距离 | `teb_local_planner` + prediction |
| **MPPI critic** | rollout 中采样障碍位置 | `ObstacleCritic` + 预测接口 |
| **VO / ORCA** | 速度空间半平面约束 | 多机/人群；常与 DWA 结合 |
| **重规划触发** | `IsPathValid` + 高频 `GetPlan` | Navigator 默认策略（非显式时空 NLP） |

Autonomy：`prediction` 与 `control` 尚未闭环；工程上可先 **Navigator 高频重规划 + MPPI 障碍 critic**，再演进 TEB 式显式时空项。详见 [Prediction 行为预测](../11_Prediction/08_behavior_prediction.md)。

### 6.7.14 Velocity Obstacles 与 kinodynamic 局部搜索

**Velocity Obstacles (VO)**：在 $(v,\omega)$ 或 $(v_x,v_y)$ 空间剔除 $T$ 秒内与动态障碍碰撞的速度集；**ORCA** 用线性约束近似，适合多智能体局部协调。

$$
\mathcal{V}_{safe} = \mathcal{V}_{max} \setminus \bigcup_j VO_j(x, \hat{o}_j, \hat{v}_j)
$$

与 DWB/MPPI 关系：DWB 在离散 $(v,\omega)$ 采样上可附加 VO 可行性过滤；MPPI 通过 **ObstacleCritic + 预测 rollout** 实现软约束。TEB 则在 band 上写**硬/软时空障碍势**。

**Kinodynamic 局部 RRT\*** / **Hybrid A\***（短 horizon）：在 $(x,t)$ 或 $(x,y,\theta,v)$ 上搜索，输出可行轨迹片段；算力高于 TEB/MPPI，多用于泊车、非结构化环境，Autonomy 未内置。全局 Hybrid A* 见 [Planning 综述 §6.2](../08_Planning/06_survey.md#62-autonomy-能力边界)。

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
| TEB | $O(n_b \cdot n_{iter})$ | 10–50 ms | 中 |
| CHOMP / TrajOpt | $O(H \cdot n_{iter})$ | 10–80 ms | 中 |
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
| 人群密集 | MPPI / TEB | SimpleGoalChecker | 高 obstacle critic；时空联合 |
| 需显式 $\Delta t$ / jerk 协调 | TEB / NMPC | StoppedGoalChecker | 见 §6.6.4 |

**Autonomy 当前建议**：FollowPath 循环完成后，**Phase 1** 上 RPP 验证闭环；**Phase 2** Graceful；**Phase 3** MPPI。Checker + Smoother 可先行单元测试。

---

## 6.15 选型决策树

```
需要沿 Path 跟踪并输出 cmd_vel？
├── 否 → 非 control 职责（behavior / manipulation）
└── 是
    ├── 环境动态障碍多？
    │   ├── 是 → 需时空联合？ → TEB / NMPC；否则 MPPI / DWB
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
| 规划-控制联合 OCP | 单 NLP 替代 Path + Controller 分层（研究多、产品少） |
| 显式时空 + 预测 | Prediction 模块 → TEB/MPPI 动态约束 |
| Semantic control | 语义地图调速（人行道/坡道） |
| 多机协调 | 速度障碍、MAPF 与 local trajectory 结合 |
| Safe RL / Diffusion traj | 学习生成 $\tau(t)$，约束校验器兜底 |

---

## 6.17 术语表

| 术语 | 英文 | 说明 |
|------|------|------|
| 局部轨迹规划 | Local Trajectory Planning | 输出 $\tau(t)$ 或 $u(t)$；Nav2 中常称 local planner / controller |
| 几何路径 | Geometric Path | $\mathcal{P}\subset SE(2)$，无时间 |
| 时空轨迹 | Spatiotemporal Trajectory | 含 $t$ 或 $\Delta t_k$ 的 $\tau$ |
| 时空联合优化 | Spatiotemporal / Kinodynamic Planning | 同时优化空间与时间（TEB、NMPC） |
| Timed Elastic Band | TEB | 位姿 + $\Delta T_k$ 交错参数化 |
| Elastic Band | EB | 仅位姿链；时间后处理 |
| Velocity Obstacle | VO / ORCA | 动态障碍速度可行集 |
| 轨迹优化 | Trajectory Optimization | CHOMP, TrajOpt, STOMP |
| Time-scaling | Time Scaling | 固定几何路径，只求 $s(t)$ |
| 轨迹跟踪 | Trajectory Tracking | 沿 Path/$\tau$ 闭环 |
| Lookahead | Look-ahead Distance | 前瞻 $L_d$ |
| 动态窗口 | Dynamic Window | 可达速度集 $\mathcal{V}_d$ |
| Critic | Critic Function | DWB/MPPI 评分项 |
| 非完整约束 | Nonholonomic | DiffDrive $v_y=0$ |
| Path Integral MPC | MPPI | 采样型随机 MPC |
| Stateful Checker | — | XY 达标后只检 yaw |

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
| 2009 | [Ratliff et al., CHOMP (ICRA)](https://doi.org/10.1109/ICRA.2009.5204119) | 梯度轨迹优化 |
| 2011 | [Kalakrishnan et al., STOMP (ICRA)](https://doi.org/10.1109/ICRA.2011.5980198) | 随机轨迹优化 |
| 2011 | [Park & Kuipers, Smooth Control (ICRA)](https://doi.org/10.1109/ICRA.2011.5980227) | Graceful 理论基础 |
| 2013 | [Zucker et al., CHOMP covariant (IJRR)](https://doi.org/10.1177/0278364912469264) | 协变轨迹优化 |
| 2014 | [Schulman et al., TrajOpt (IJRR)](https://doi.org/10.1177/0278364914528137) | 序列凸轨迹优化 |
| 2017 | [Rösmann et al., TEB (IEEE RAM)](https://doi.org/10.1109/MRA.2016.2582927) | 时空弹性带 |
| 1998 | [Fiorini & Shiller, VO (IJRR)](https://doi.org/10.1177/027836498901800402) | 速度障碍 |
| 2011 | [van den Berg et al., ORCA (IROS)](https://doi.org/10.1109/IROS.2011.6096362) | 多智能体局部避障 |
| 2017 | [Williams et al., IT-MPC (ICRA)](https://ieeexplore.ieee.org/document/7989202) | MPPI |
| 2022 | [Macenski et al., Nav2 (Science Robotics)](https://www.science.org/doi/10.1126/scirobotics.abm6074) | 工程栈 |

**工程**

- [Navigation2 Controllers](https://docs.nav2.org/configuration/packages/configuring-controller-server.html)
- [nav2_mppi_controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller)
- [nav2_graceful_controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_graceful_controller)

---

## 6.19 相关文档

- [§0 指南](00_guide.md) · [§2 架构](02_architecture.md)
- [§3 Checkers](03_checkers.md) · [§4 Smoother](04_velocity_smoother.md) · [§5 控制器总览](05_controller_algorithms.md#581-控制器与-checker-配对)
- [Planning 综述](../08_Planning/06_survey.md)（全局几何 · [§6.5.4](../08_Planning/06_survey.md#654-采样优化与反应式未内置)） · [Navigator 综述](../16_Navigator/06_survey.md)
- [Prediction 行为预测](../11_Prediction/08_behavior_prediction.md)（动态障碍 · TEB/MPPI 扩展）
