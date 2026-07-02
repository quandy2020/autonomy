(dwb-controller)=
# 13. DWB Controller

> 归属 [§8 局部控制器 · §8.5](../08_controller_algorithms.md#85-dwbdynamic-window-benchmark)
>
> **Dynamic Window Benchmark**（DWB）是 Nav2 默认局部控制器 `nav2_dwb_controller`：在 David Lu 可插拔 Critic 框架上，将 Fox (1997) **DWA**（动态窗口法）工程化落地。
>
> **解决什么问题**：全局规划器给出参考路径，但机器人仍需在**运动学约束**与**动态障碍**下，每周期（约 20 Hz）决定下一步的线速度与角速度。DWB 不搜索整条路径，而是在当前时刻的**合法速度空间** $\mathcal{V}_{legal}$（可达 ∩ 可刹停 ∩ 电机极限）内网格采样，对每组 $(v,\omega)$ 前向仿真短轨迹，经 Critics 加权打分后输出最优 `cmd_vel`。
>
> **为何选用 DWB**：单周期计算轻（典型 1–5 ms）、轨迹可解释、Critic 插件可按场景组合调参；Nav2 默认方案，适合差速/全向底盘的跟径与实时避障。Autonomy 侧尚未实现，本文作移植与调参参考。
>
> 公共推导 [03_math.md §3.7](../03_math.md) · 算法对比 [09_survey.md §9.5.4](../09_survey.md)

| 维度 | 说明 |
|------|------|
| 类型 | 速度采样 + 轨迹 rollout + Critic 加权 |
| 输入 | 路径、位姿、速度、costmap |
| 输出 | $(v_x^{cmd}, \omega_z^{cmd})$ |
| Autonomy | ❌ 未实现（设计/移植参考） |

**文档导航**

| 章节 | 内容 | 适合读者 |
|------|------|----------|
| [§1 速览](#1-速览) | 流水线 + **算法（LaTeX）** | 快速入门 |
| [§2 数学](#2-数学原理) | 三阶段公式 | 算法推导 |
| [§3 架构](#3-nav2-架构) | 插件与数据流 | 工程实现 |
| [§4 Critics](#4-critics) | Critic 数学原理 | 调参 |
| [§5 配置](#5-配置与调参) | 参数与排错 | 集成调试 |
| [§6 移植](#6-autonomy-移植) | checklist | Autonomy 开发 |
| [§7 文献](#7-参考文献) | 必读论文 | 深入研究 |

---

## 1. 速览

### 1.1 核心思想

每控制周期求解：

$$
(v^*, \omega^*) = \arg\min_{(v,\omega) \in \mathcal{V}_{legal}} C_{total}\!\big(\tau(v,\omega)\big)
$$

$\mathcal{V}_{legal}$ = 电机可达 ∩ 动态窗口 ∩ 可安全刹车；$\tau$ = 前向仿真轨迹。

### 1.2 三阶段流水线

单控制周期内，数据沿 **A → B → C** 单向流动；每阶段输出即下一阶段输入。

<div class="plan-arch-diagram">

  <div class="plan-arch-layer plan-arch-app">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">阶段 A</span>
      <span class="plan-arch-title">搜索空间</span>
      <div class="plan-arch-sub">构造 <span class="math notranslate nohighlight">\(\mathcal{V}_{legal}\)</span> 并网格采样 · <code>TrajectoryGenerator</code></div>
    </div>
    <div class="plan-arch-body plan-arch-body-cols">
      <div class="nav-body-block">
        <div class="nav-body-label">核心步骤</div>
        <div class="nav-chip-list">
          <span class="nav-chip">A.1 三集合交集</span>
          <span class="nav-chip">A.2 网格采样</span>
        </div>
      </div>
      <div class="nav-body-block">
        <div class="nav-body-label">数据流</div>
        <ul>
          <li><strong>入</strong> <span class="math notranslate nohighlight">\((v_c,\omega_c)\)</span>，costmap</li>
          <li><strong>出</strong> <span class="math notranslate nohighlight">\(\{(v_i,\omega_i)\}_{i=1}^{K}\)</span>，<span class="math notranslate nohighlight">\(K=N_v N_\omega\)</span></li>
        </ul>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>候选速度 <span class="math notranslate nohighlight">\(\{(v_i,\omega_i)\}\)</span></span></div>

  <div class="plan-arch-layer plan-arch-server">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">阶段 B</span>
      <span class="plan-arch-title">轨迹预测</span>
      <span class="plan-arch-sub">前向 rollout · <code>TrajectoryGenerator</code> + <code>Rollout</code></span>
    </div>
    <div class="plan-arch-body plan-arch-body-cols">
      <div class="nav-body-block">
        <div class="nav-body-label">核心步骤</div>
        <div class="nav-chip-list">
          <span class="nav-chip">B.1 运动学模型</span>
          <span class="nav-chip">B.2 积分 + rollout</span>
        </div>
      </div>
      <div class="nav-body-block">
        <div class="nav-body-label">数据流</div>
        <ul>
          <li><strong>入</strong> 每组 <span class="math notranslate nohighlight">\((v_i,\omega_i)\)</span></li>
          <li><strong>出</strong> 轨迹集 <span class="math notranslate nohighlight">\(\{\tau_1,\ldots,\tau_K\}\)</span></li>
        </ul>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>轨迹 <span class="math notranslate nohighlight">\(\{\tau_i\}\)</span> 与关联速度</span></div>

  <div class="plan-arch-layer plan-arch-map">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">阶段 C</span>
      <span class="plan-arch-title">评价选优</span>
      <span class="plan-arch-sub">Critics 加权 + <span class="math notranslate nohighlight">\(\arg\min\)</span> · <code>TrajectoryCritic</code> 链</span>
    </div>
    <div class="plan-arch-body plan-arch-body-cols">
      <div class="nav-body-block">
        <div class="nav-body-label">核心步骤</div>
        <div class="nav-chip-list">
          <span class="nav-chip">C.1 分项代价</span>
          <span class="nav-chip">C.2 加权合并</span>
          <span class="nav-chip">C.3 argmin 输出</span>
        </div>
      </div>
      <div class="nav-body-block">
        <div class="nav-body-label">数据流</div>
        <ul>
          <li><strong>入</strong> <span class="math notranslate nohighlight">\(\{\tau_i\}\)</span>，<span class="math notranslate nohighlight">\(\mathcal{P}\)</span>，<span class="math notranslate nohighlight">\(\mathcal{C}\)</span></li>
          <li><strong>出</strong> <span class="math notranslate nohighlight">\((v^*,\omega^*)\)</span> → <code>cmd_vel</code></li>
        </ul>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe plan-arch-pipe-out"><span>下发速度指令 cmd_vel</span></div>

</div>

| 阶段 | Nav2 组件 | 详见 |
|------|-----------|------|
| A 搜索空间 | `TrajectoryGenerator`（前半） | [§1.3](#13-算法数学描述) · [§2.1](#21-阶段-a合法速度空间) |
| B 轨迹预测 | `TrajectoryGenerator`（rollout） | [§1.3 算法 2](#13-算法数学描述) · [§2.2](#22-阶段-b轨迹预测) |
| C 评价选优 | `TrajectoryCritic[]` | [§1.3 算法 3](#13-算法数学描述) · [§4](#4-critics) |

### 1.3 算法（数学描述）

<div class="algorithm-box-diagram">

<div class="algorithm-box algorithm-box-phase-a">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 1</span>
    <span class="algorithm-box-title">DWB 局部规划器</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{DWB}(\mathbf{x}, v_c, \omega_c, \mathcal{P}, \mathcal{C}, \mathcal{M}; \Theta) \mapsto (v^*, \omega^*)$

  </div>
  <div class="algorithm-box-io" markdown="1">

| 方向 | 符号 | 说明 |
|------|------|------|
| 输入 | $\mathbf{x}$ | 机器人位姿 $(x,y,\theta)$ |
| 输入 | $(v_c,\omega_c)$ | 当前线速度、角速度 |
| 输入 | $\mathcal{P}$ | 全局参考路径 |
| 输入 | $\mathcal{C}$ | 局部代价地图（costmap） |
| 输入 | $\mathcal{M}=\{c_m\}_{m=1}^{M}$ | TrajectoryCritic 插件集 |
| 输入 | $\Theta$ | 算法参数（$N_v,N_\omega,T_{\mathrm{sim}},\Delta t_{\mathrm{sim}},\ldots$） |
| 输出 | $(v^*,\omega^*)$ | 本周期最优速度指令（`cmd_vel`） |

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&\mathcal{V}_s \leftarrow [v_{\min}, v_{\max}] \times [\omega_{\min}, \omega_{\max}]
&\qquad\text{（电机能力）} \\[6pt]
\textbf{2.}\;&\mathcal{V}_d \leftarrow \big\{(v,\omega) : v \in [v_c - \dot{v}_{\mathrm{dec}}\,dt,\, v_c + \dot{v}_{\mathrm{acc}}\,dt],\;
\omega \in [\omega_c - \dot{\omega}_{\mathrm{dec}}\,dt,\, \omega_c + \dot{\omega}_{\mathrm{acc}}\,dt]\big\}
&\qquad\text{（动态窗口）} \\[6pt]
\textbf{3.}\;&\mathcal{V}_a \leftarrow \big\{(v,\omega) : v \leq \sqrt{2\,d\,\dot{v}_{\mathrm{brake}}},\;
\omega \leq \sqrt{2\,d_\theta\,\dot{\omega}_{\mathrm{brake}}}\big\}
&\qquad\text{（刹车安全）} \\[6pt]
\textbf{4.}\;&\mathcal{V}_{\mathrm{legal}} \leftarrow \mathcal{V}_s \cap \mathcal{V}_d \cap \mathcal{V}_a \\[6pt]
\textbf{5.}\;&\mathcal{C}_{\mathrm{vel}} \leftarrow \mathrm{GridSample}(\mathcal{V}_{\mathrm{legal}}, N_v, N_\omega)
&\qquad\text{（}K = N_v N_\omega\text{ 组候选速度）} \\[6pt]
\textbf{6.}\;&\mathcal{T} \leftarrow \emptyset \\[6pt]
\textbf{7.}\;&\textbf{for each } (v_i, \omega_i) \in \mathcal{C}_{\mathrm{vel}} \textbf{ do} \\[3pt]
&\quad \tau_i \leftarrow \mathrm{Rollout}(\mathbf{x}, v_i, \omega_i, T_{\mathrm{sim}}, \Delta t_{\mathrm{sim}})
\qquad\text{（子程序见算法 2）} \\[3pt]
&\quad \mathcal{T} \leftarrow \mathcal{T} \cup \{(v_i, \omega_i, \tau_i)\} \\[6pt]
\textbf{8.}\;&(v^*, \omega^*) \leftarrow \underset{(v_i,\omega_i,\tau_i) \in \mathcal{T}}{\arg\min}\;
\mathrm{Score}(\tau_i, v_i, \mathcal{P}, \mathcal{C}, \mathcal{M})
\qquad\text{（子程序见算法 3）} \\[6pt]
\textbf{9.}\;&\textbf{return } (v^*, \omega^*)
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box algorithm-box-phase-b algorithm-box-subroutine">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 2</span>
    <span class="algorithm-box-title">Rollout 子程序</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{Rollout}(\mathbf{x}, v, \omega, T_{\mathrm{sim}}, \Delta t) \to \tau$

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&(x, y, \theta) \leftarrow \mathbf{x};\quad \tau \leftarrow \emptyset;\quad t \leftarrow 0 \\[6pt]
\textbf{2.}\;&\textbf{while } t < T_{\mathrm{sim}} \textbf{ do} \\[3pt]
&\quad (x, y, \theta) \leftarrow \Phi(x, y, \theta, v, \omega, \Delta t)
\qquad\text{（}\Phi\text{：欧拉或圆弧积分，见 §2.2）} \\[3pt]
&\quad \tau \leftarrow \tau \cup \{(x, y, \theta)\};\quad t \leftarrow t + \Delta t \\[6pt]
\textbf{3.}\;&\textbf{return } \tau
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box algorithm-box-phase-c algorithm-box-subroutine">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 3</span>
    <span class="algorithm-box-title">Score 子程序</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{Score}(\tau, v, \mathcal{P}, \mathcal{C}, \mathcal{M}) \to S$

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&S \leftarrow 0 \\[6pt]
\textbf{2.}\;&\textbf{for each } c_m \in \mathcal{M} \textbf{ do} \\[3pt]
&\quad s_m \leftarrow c_m.\mathrm{score}(\tau, v, \mathcal{P}, \mathcal{C}) \\[3pt]
&\quad \textbf{if } s_m = +\infty \textbf{ then return } +\infty
\qquad\text{（碰撞短路截断）} \\[3pt]
&\quad S \leftarrow S + w_m \cdot s_m \\[6pt]
\textbf{3.}\;&\textbf{return } S
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box-footer" markdown="1">

$w_m$ 为 Critic 权重（Nav2 `scale`）。**复杂度** $O(K \cdot N \cdot M)$：$K = N_v N_\omega$，$N = T_{\mathrm{sim}} / \Delta t_{\mathrm{sim}}$，$M = |\mathcal{M}|$。典型 **1–5 ms** / 周期。

</div>

</div>

---

## 2. 数学原理

> 按 **A → B → C** 阅读；公式细节与 [§1.3](#13-算法数学描述) 算法步骤一一对应。

### 2.1 阶段 A：合法速度空间

$$
\mathcal{V}_{legal} = \mathcal{V}_s \cap \mathcal{V}_d \cap \mathcal{V}_a
$$

| 集合 | 含义 | 公式要点 |
|------|------|----------|
| $\mathcal{V}_s$ | 电机能力 | $v \in [v_{min},v_{max}],\; \omega \in [\omega_{min},\omega_{max}]$ |
| $\mathcal{V}_d$ | 动态窗口 | $v \in [v_c - \dot{v}_{dec}\,dt,\; v_c + \dot{v}_{acc}\,dt]$（$\omega$ 同理） |
| $\mathcal{V}_a$ | 刹车安全 | $v \leq \sqrt{2\,d\,\dot{v}_{brake}}$（$\omega$ 用 $d_\theta$） |

网格采样（$K = N_v \times N_\omega$）：

$$
v_i = v_{min}^{legal} + i \cdot \frac{v_{max}^{legal} - v_{min}^{legal}}{N_v - 1}, \quad
\omega_j = \omega_{min}^{legal} + j \cdot \frac{\omega_{max}^{legal} - \omega_{min}^{legal}}{N_\omega - 1}
$$

Nav2：`vx_samples=20`，`vtheta_samples=20`，全向加 `vy_samples=5`。

### 2.2 阶段 B：轨迹预测

**连续模型**（分段恒定 $v,\omega$）：

$$
\dot{x} = v\cos\theta,\quad \dot{y} = v\sin\theta,\quad \dot{\theta} = \omega
$$

**离散积分**（二选一）：

| 方法 | 公式 | 适用 |
|------|------|------|
| 欧拉 | $x_{t+1} = x_t + v\cos\theta_t \Delta t$（$y,\theta$ 同理） | $\Delta t$ 极小 |
| 圆弧 | 见下式；$\omega\to0$ 退化为直线 | $\omega$ 较大 |

圆弧积分（$\omega \neq 0$，$R=v/\omega$）：

$$
\begin{bmatrix} x_{t+1} \\ y_{t+1} \\ \theta_{t+1} \end{bmatrix}
= \begin{bmatrix} x_t \\ y_t \\ \theta_t \end{bmatrix}
+ \begin{bmatrix}
-\frac{v}{\omega}\sin\theta_t + \frac{v}{\omega}\sin(\theta_t+\omega\Delta t) \\
\frac{v}{\omega}\cos\theta_t - \frac{v}{\omega}\cos(\theta_t+\omega\Delta t) \\
\omega\Delta t
\end{bmatrix}
$$

Rollout：迭代 $N = \lfloor T_{sim}/\Delta t_{sim}\rfloor$ 步得 $\tau_i$。典型 `sim_time=1.7` s。

### 2.3 阶段 C：评价与输出

**DWA 三项**（Fox 1997，Nav2 扩展为 Critic 插件）：

| 分项 | 公式 | DWB Critic |
|------|------|------------|
| 航向 | $\|\mathrm{AngleDiff}(\theta_N, \theta_{target})\|/\pi$ | PathAlign, GoalAlign |
| 避障 | $\infty$ 若碰撞；否则 $1/dist_{min}$ | ObstacleFootprint |
| 速度 | $v_{max} - v$ | PreferForward |

**合并**（见 [§4.1](#41-统一数学框架) 各 Critic 公式）：

$$
C_{total} = \sum_{m} w_m \cdot c_m(\tau), \quad
(v^*,\omega^*) = \arg\min C_{total}, \quad
\text{cmd_vel} = (v^*, \omega^*)
$$

---

## 3. Nav2 架构

```
ControllerServer → DWBLocalPlanner
                      ├─ TrajectoryGenerator   // 阶段 A + B
                      └─ TrajectoryCritic[]    // 阶段 C（.so 插件）
```

| Nav2 组件 | 职责 | Autonomy 对应 |
|-----------|------|---------------|
| `DWBLocalPlanner` | 调度 A→B→C | `ControllerInterface` 子类 ⏳ |
| `TrajectoryGenerator` | 采样 + rollout；支持 DiffDrive / Omni | 内部模块 ⏳ |
| `TrajectoryCritic` | 可插拔打分；碰撞可短路 | Critic 接口 ⏳ |
| `Costmap2D` | 障碍查询 | `Costmap2DWrapper` ✅ |
| `GoalChecker` | 到达判定 | `GoalChecker` ✅ |

**策略模式要点**

- **TrajectoryGenerator**：`StandardTrajectoryGenerator`；欧拉/圆弧积分；$(v_x,v_y,\omega_z)$ 全向采样
- **TrajectoryCritic**：yaml 组合 + `scale`；`ObstacleFootprint` 多边形足迹 + 短路；`Oscillation` 跨周期抑振

---

## 4. Critics

> Critic 是算法 3 中 $\mathrm{Score}$ 的逐项实现。轨迹 $\tau = \{(x_k,y_k,\theta_k)\}_{k=0}^{N}$，终点 $\mathbf{p}_N=(x_N,y_N,\theta_N)$，全局路径 $\mathcal{P}=\{p_j\}_{j=1}^{L}$，目标 $\mathbf{g}=(x_g,y_g,\theta_g)$。

### 4.1 统一数学框架

每条轨迹的总代价（最小化）：

$$
C_{total}(\tau, v) = \sum_{m=1}^{M} w_m \cdot c_m(\tau, v)
$$

| 符号 | 含义 | Nav2 参数 |
|------|------|-----------|
| $c_m$ | 第 $m$ 个 Critic 原始得分 | 插件 `score()` 返回值 |
| $w_m$ | 权重 | `CriticsName.scale` |
| $+\infty$ | 碰撞/非法轨迹 | 触发**短路截断**，跳过后续 Critic |

**评分点约定**（Lu 2014 / Nav2 默认）：

| 类型 | 采样方式 |
|------|----------|
| 障碍类 | 沿 $\tau$ **全程**或稀疏采样各点 $(x_k,y_k,\theta_k)$ |
| 路径/目标类 | 主要用**轨迹终点** $\mathbf{p}_N$，PathAlign/GoalAlign 用前向点 $\mathbf{p}_{fwd}$ |

前向点（`forward_point_distance = d_{fwd}`）：

$$
\mathbf{p}_{fwd} = \mathbf{p}_N + d_{fwd}\begin{bmatrix}\cos\theta_N \\ \sin\theta_N\end{bmatrix}
$$

角度差统一记为：

$$
\Delta\theta(a,b) = \big|\mathrm{AngleDiff}(a,\, b)\big| \in [0,\pi]
$$

### 4.2 与 DWA 三项的映射

| DWA 分项 (Fox 1997) | 数学形式 | DWB Critic |
|---------------------|----------|------------|
| heading | $\Delta\theta(\theta_N, \theta_{target})/\pi$ | PathAlign, GoalAlign, GoalDist |
| clearance | $\infty$ 或 $1/d_{min}$ | ObstacleFootprint, BaseObstacle |
| velocity | $v_{max}-v$ | PreferForward, Twirling |

经典 DWA 对三项做批次归一化；Nav2 用各 `scale` 等效加权（见 §2.3）。

### 4.3 逐 Critic 公式

#### 4.3.1 障碍类

**BaseObstacleCritic** — 栅格代价累积（快，圆形机器人）：

$$
c_{base}(\tau) = \sum_{k=0}^{N} C\!\big(x_k, y_k\big)
$$

$C(x,y)$ 为 costmap 代价值（0=自由，254=致死）。亦可用终点单点 $C(x_N,y_N)$ 以省算力。

**ObstacleFootprintCritic** — 多边形足迹（精确，可短路）：

沿轨迹检测机器人足迹 $\mathcal{F}(x_k,y_k,\theta_k)$（矩形或多边形）：

$$
c_{fp}(\tau) = \begin{cases}
+\infty & \exists k:\; \mathcal{F}_k \cap \mathcal{O} \neq \emptyset \\[4pt]
\displaystyle\sum_{k} \frac{1}{d_{min}(\mathcal{F}_k,\,\mathcal{O}) + \epsilon} & \text{otherwise}
\end{cases}
$$

$\mathcal{O}$ 为障碍集合，$d_{min}$ 为足迹到最近障碍距离。碰撞时 $c_{fp}=+\infty$，算法 3 立即返回。

#### 4.3.2 路径跟踪类

**PathDistCritic** — 终点偏离全局路径：

$$
c_{pd}(\tau) = \min_{p \in \mathcal{P}} \|\mathbf{p}_N - p\|_2
$$

**PathAlignCritic** — 前向点与路径切向对齐（防抄近道）：

设路径上距 $\mathbf{p}_{fwd}$ 最近点为 $p^*$，切向角 $\theta_{\mathcal{P}}(p^*)$：

$$
c_{pa}(\tau) = \Delta\theta\!\big(\theta_N,\; \theta_{\mathcal{P}}(p^*)\big)
\quad\text{或}\quad
\|\mathbf{p}_{fwd} - p^*\|_2
$$

Nav2 实现以**角度差**为主；`forward_point_distance` 控制前向点距离。

#### 4.3.3 目标趋近类

**GoalDistCritic** — 终点距目标位置：

$$
c_{gd}(\tau) = \|\mathbf{p}_N - (x_g, y_g)\|_2
$$

**GoalAlignCritic** — 终点航向与目标航向一致：

$$
c_{ga}(\tau) = \Delta\theta(\theta_N,\; \theta_g)
$$

通常与 GoalDist 联用：远距离 GoalDist 主导，近目标 GoalAlign 主导。

**RotateToGoalCritic** — 近目标减速并鼓励原地转向：

当 $\|(x,y)-(x_g,y_g)\| < \varepsilon_{xy}$ 时激活。设仿真末端线速度 $v_N$：

$$
c_{rg}(\tau) = \frac{\|v_N\|}{\sigma_{slow}} \cdot \mathbb{1}_{\text{near goal}}
$$

$\sigma_{slow}$ 对应 `slowing_factor`；近目标时惩罚平移、保留转向自由度，与 `GoalChecker` 协同。

#### 4.3.4 行为约束类

**PreferForwardCritic** — 惩罚后退：

$$
c_{pf}(v) = \max(0,\; -v_x) \cdot w_{pf}
$$

（若 Critic 内部未含权重，则 $w_{pf}$  absorbed 进 `scale`。）

**TwirlingCritic** — 抑制低线速度高空速度（原地空转）：

$$
c_{tw}(\tau) = |\omega| \cdot \mathbb{1}_{\|v\| < v_{thresh}}
$$

**OscillationCritic** — 跨周期振荡检测（状态记忆）：

维护历史速度符号 $\mathrm{sign}(v_x^{(t-1)}), \mathrm{sign}(\omega^{(t-1)})$。若当前轨迹首步速度符号与历史呈**往复模式**（如左右反复扭动）：

$$
c_{osc}(\tau) = \mathbb{1}_{\text{oscillating}} \cdot C_{osc}^{max}
$$

$C_{osc}^{max}$ 为极大惩罚，迫使 planner 选择打破振荡的轨迹。

### 4.4 Critics 协作时序

导航过程中各 Critic **空间主导权**随距目标远近切换：

```
远离目标                         接近目标
├─ PathDist / PathAlign（贴路径） ├─ GoalDist / GoalAlign（进目标）
├─ BaseObstacle / Footprint       ├─ RotateToGoal（原地转向）
├─ PreferForward                  ├─ Oscillation（抑振）
└─ 全程：障碍 Critic 始终生效
```

**推荐加载顺序**（障碍前置 + `short_circuit_trajectory_evaluation: true`）：

```yaml
critics: [ObstacleFootprint, PathDist, PathAlign, GoalDist, GoalAlign, Oscillation, RotateToGoal]
```

### 4.5 权重 scale 参考

Nav2 默认示例（需按场景整定）：

| Critic | 典型 scale | 作用 |
|--------|------------|------|
| BaseObstacle | 0.02 | 避障（栅格） |
| PathDist / PathAlign | 32.0 | 路径跟踪 |
| GoalDist / GoalAlign | 24.0 | 目标趋近 |
| RotateToGoal | 32.0 | 近目标转向 |
| PreferForward | 1–10 | 禁止倒车 |

| 现象 | 调整 |
|------|------|
| 贴障 | 增大 Obstacle `scale`；换 Footprint |
| 偏离路径 | 增大 PathDist / PathAlign |
| 目标摆动 | 启用 Oscillation；略降 Goal scale |
| 近目标冲过头 | 增大 RotateToGoal `slowing_factor` |
| 原地打转 | 启用 Twirling；检查 PathAlign |

---

## 5. 配置与调参

| 参数 | 典型值 | 对应 |
|------|--------|------|
| `sim_time` | 1.7 s | $T_{sim}$ |
| `vx_samples` / `vtheta_samples` | 20 / 20 | $N_v$, $N_\omega$ |
| `linear_granularity` | 0.05 m | rollout 步长 |
| `acc_lim_x` / `decel_lim_x` | 2.5 / -2.5 m/s² | $\mathcal{V}_d$, $\mathcal{V}_a$ |
| `max_vel_x` / `max_vel_theta` | 0.5 m/s / 1.0 rad/s | $\mathcal{V}_s$ |
| `short_circuit_trajectory_evaluation` | true | 碰撞短路 |
| `forward_point_distance` | 0.1 m | $d_{fwd}$（Path/GoalAlign） |

Critic 权重与排错详见 [§4.5](#45-权重-scale-参考)。

| 现象 | 调整 |
|------|------|
| 反应慢 | 增 `vx_samples` 或 `sim_time` |
| 超时 | 减 samples 或增大 granularity |

---

## 6. Autonomy 移植

```
□ ControllerInterface 子类
□ TrajectoryGenerator（𝒱_legal + Rollout）
□ TrajectoryCritic 插件接口
□ Costmap2DWrapper 足迹检测
□ controller.lua 配置 + 插件注册
□ 测试：直线 / 绕障 / 窄道 / 目标旋转
```

---

## 7. 参考文献

### 7.1 必读

| # | 文献 | 贡献 | 链接 |
|---|------|------|------|
| ① | Fox et al., 1997 — *DWA* | 动态窗口 + 三项代价 | [IEEE](https://ieeexplore.ieee.org/document/619666) |
| ② | Lu, 2014 — PhD thesis | Critic 可插拔（Appendix A） | [DOI](https://doi.org/10.7936/K77P8WHT) |
| ③ | Macenski et al., 2020 — *Marathon 2* | Nav2 架构 | [DOI](https://doi.org/10.1109/IROS45743.2020.9341207) |
| ④ | Urrea, 2024+ | DWB vs MPPI/RPP 评测 | 农业/半结构化场景 |

### 7.2 演进

```
1997 DWA → 2014 Lu(Critic插件) → 2016+ dwb_local_planner → 2020 Nav2 → 2024+ 工程评测
```

### 7.3 资源

| 资源 | 链接 |
|------|------|
| Nav2 DWB 配置 | [configuring-dwb-controller](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html) |
| 源码 | [nav2_dwb_controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_dwb_controller) |

```bibtex
@article{fox1997dynamic,
  author  = {Fox, Dieter and Burgard, Wolfram and Thrun, Sebastian},
  title   = {The Dynamic Window Approach to Collision Avoidance},
  journal = {IEEE Robotics \& Automation Magazine},
  volume  = {4}, number = {1}, pages = {23--33}, year = {1997}
}
```
