(mppi-controller)=
# 11. MPPI Controller

> 归属 [§5.3 MPPI](../05_controller_algorithms.md#53-mppi-controller) · Autonomy ⏳ 配置预留，插件未实现
>
> **Model Predictive Path Integral**（MPPI，Nav2 `nav2_mppi_controller`）在预测时域 $H$ 上对 $K$ 条扰动控制序列 batch 前向仿真，以 Critic 链累加轨迹代价 $S^{(k)}$，再经 **softmax 加权**得 $\mathbf{u}^*$ 并执行首步 $\mathbf{u}_0^*$；与 §15 **NMPC**（显式 OCP 求解）同属滚动预测控制，但无 NLP/QP 求解器。

---

## 1. 背景

局部控制需在非凸、非光滑代价（costmap、路径偏差、目标）下做**前瞻避障与跟踪**。**Dynamic Window Approach**（Fox, 1997）每周期仅优化单步 $(v,\omega)$；**模型预测控制**延长时域，但经典 NMPC 依赖 SQP/QP。**路径积分 / 信息论 MPC**（Theodorou et al., 2010 PI²；Williams et al., 2017 ICRA）将最优控制写为轨迹代价的指数加权期望；**MPPI**（Williams et al., 2018 IROS）在 receding-horizon 下 batch 采样 + softmax，适合 GPU/向量化并行。Nav2 `nav2_mppi_controller` 以 Critic 插件实现 $q(\mathbf{x},\mathbf{u})$，默认 $K=2000$、$H=56$、$\Delta t=0.05$ s（详见 §3–§5）。

---

## 2. 问题

**任务.** 平面移动机器人沿参考路径 $\mathcal{P}$ 向局部目标前进，在 costmap $\mathcal{C}$ 下输出本周期速度指令。

**输入 / 输出.** 位姿 $\mathbf{x}_0$、当前速度、路径 $\mathcal{P}$、代价地图 $\mathcal{C}$、Critic 集 $\mathcal{M}$ → $(v_x^{cmd},\, v_y^{cmd},\,\omega_z^{cmd})$（差速时 $v_y=0$）。

**在线形式.** **滚动时域 MPPI**：每周期对 $K$ 条控制序列 $\mathbf{U}^{(k)}=\{\mathbf{u}_0^{(k)},\ldots,\mathbf{u}_{H-1}^{(k)}\}$ 仿真、打分、softmax 得 $\mathbf{U}^*$，执行 $\mathbf{u}_0^*$；下一周期 $\bar{\mathbf{U}}$ 左移 warm-start（Williams 2018；Nav2 `shift_control_sequence`）。要求 $f_{ctrl}=1/\Delta t$。

---

## 3. 运动模型

以下给出 §4 批量 rollout 所需链条：**连续运动学 → 离散 $H$ 步预测 → 轨迹张量 $\mathbf{X}^{(k)}$**。

### 3.1 差速连续运动学

世界系 $\{W\}$ 下 $\mathbf{x}=[x,y,\theta]^\top$，控制 $\mathbf{u}=[v_x,\,\omega_z]^\top$（Fox / Nav2 记号）：

$$
\dot{x}=v_x\cos\theta,\quad \dot{y}=v_x\sin\theta,\quad \dot{\theta}=\omega_z.
$$

- **含义**：预测模型；$\mathbf{u}_t$ 在 $[\Delta t,\,2\Delta t)$ 内分段恒定。

**全向**（Nav2 `Omni`）：$\mathbf{u}=[v_x,v_y,\omega_z]^\top$，$\dot{x}=v_x\cos\theta-v_y\sin\theta$，$\dot{y}=v_x\sin\theta+v_y\cos\theta$。

**Ackermann**：$|v_x|\leq |\omega_z|\,R_{\min}$（`min_turning_r`），采样后 clip。

### 3.2 离散 Rollout

给定 $\mathbf{U}^{(k)}$，从 $\mathbf{x}_0$ 欧拉积分 $H$ 步（Nav2 `MotionModel`；与 [DWB §3.2](13_dwb_controller.md#32-离散-rollout) 同型）：

$$
\mathbf{x}_{t+1}^{(k)} = \mathbf{x}_t^{(k)} +
\begin{bmatrix} v_x^{(k)}\cos\theta_t^{(k)} \\ v_x^{(k)}\sin\theta_t^{(k)} \\ \omega_z^{(k)} \end{bmatrix}\Delta t.
$$

- **$\mathbf{X}^{(k)}=\{\mathbf{x}_t^{(k)}\}_{t=0}^{H}$**：第 $k$ 条候选轨迹；Critics 沿 $\mathbf{X}^{(k)}$ 采样评估（非单点 $\mathbf{x}_N$）。
- **硬界**：$\mathbf{u}_t^{(k)}\leftarrow\mathrm{clip}(\bar{\mathbf{u}}_t+\boldsymbol{\epsilon}_t^{(k)},\,\mathcal{U}_{limits})$，$\boldsymbol{\epsilon}_t^{(k)}\sim\mathcal{N}(0,\Sigma)$。

---

## 4. 数学问题定义

每周期在名义序列 $\bar{\mathbf{U}}$ 邻域采样 $K$ 条 $\mathbf{U}^{(k)}$，最小化路径积分代价的 Monte Carlo 近似；§4.1 定义 $S(\mathbf{U})$，§4.2 给出 softmax 控制律（Williams 2017；Nav2 `Optimizer`）。

### 4.1 路径积分代价

**随机最优控制**（Williams 2017 信息论 MPC）。第 $k$ 条采样序列总代价：

$$
S^{(k)} = S(\mathbf{U}^{(k)}) =
\sum_{t=0}^{H-1}
\Big(
q\big(\mathbf{x}_t^{(k)},\, \mathbf{u}_t^{(k)}\big)
+ \frac{\gamma}{2}\,
{\mathbf{u}_t^{(k)}}^\top \Sigma^{-1} \bar{\mathbf{u}}_t
\Big).
$$

- **$q$**：运行代价，Nav2 为 Critic 加权和 $q=\sum_m w_m c_m(\mathbf{X}^{(k)},\mathbf{U}^{(k)},\mathcal{P},\mathcal{C})$（PathAlign、CostCritic、Goal 等；与 [DWB §4.2](13_dwb_controller.md#42-速度决策主问题) 语义相近，但作用于**整段** $\mathbf{X}^{(k)}$）。
- **$\gamma$**：控制正则（Nav2 `gamma`）；**$\Sigma=\mathrm{diag}(\sigma_{v_x}^2,\sigma_{v_y}^2,\sigma_{\omega_z}^2)$**：`vx_std` / `vy_std` / `wz_std`。
- **$\bar{\mathbf{U}}$**：warm-start 名义序列；采样 $\mathbf{u}_t^{(k)}=\mathrm{clip}(\bar{\mathbf{u}}_t+\boldsymbol{\epsilon}_t^{(k)})$。
- **碰撞**：CostCritic 等可令 $S^{(k)}\to\infty$（`collision_cost`），等价硬排除。

### 4.2 MPPI 控制律（softmax 主问题）

Williams (2017) 路径积分 / 自由能最优控制 → Monte Carlo softmax（Nav2 `updateControlSequence`）：

$$
\mathbf{u}_t^* = \sum_{k=1}^{K} w_k\, \mathbf{u}_t^{(k)}, \qquad
w_k = \frac{\exp\big(-(S^{(k)}-S_{\min})/\lambda\big)}{\sum_{j=1}^{K}\exp\big(-(S^{(j)}-S_{\min})/\lambda\big)}.
$$

- **$\lambda$**：温度（Nav2 `temperature`）；$\lambda\to 0$ 权重集中于最低代价轨迹，$\lambda\to\infty$ 趋于均匀。
- **$S_{\min}=\min_k S^{(k)}$**：数值稳定，不改变 $w_k$ 比例。
- **滚动输出**：执行 $\mathbf{u}_0^*$；warm-start $\bar{\mathbf{u}}_t\leftarrow \mathbf{u}_{t+1}^*$，$t=0,\ldots,H-2$，末步补噪声（IROS 2018 receding horizon）。

---

## 5. 求解

§4 在单周期内按 **warm-start → batch 采样 rollout → Critic 打分 → softmax → 输出 $\mathbf{u}_0^*$** 执行；算法 1–3 与 Nav2 `Optimizer` 对应。

### 5.1 算法（数学描述）

<div class="algorithm-box-diagram">

<div class="algorithm-box algorithm-box-phase-a">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 1</span>
    <span class="algorithm-box-title">MPPI 局部控制器</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{MPPI}(\mathbf{x}_0, \mathbf{v}_c, \mathcal{P}, \mathcal{C}, \mathcal{M}, \bar{\mathbf{U}};\, \Theta) \mapsto \mathbf{u}_0^*$

  </div>
  <div class="algorithm-box-io" markdown="1">

| 方向 | 符号 | 说明 |
|------|------|------|
| 输入 | $\mathbf{x}_0$ | 当前位姿 |
| 输入 | $\mathbf{v}_c$ | 当前速度（闭环） |
| 输入 | $\mathcal{P},\,\mathcal{C}$ | 路径、costmap |
| 输入 | $\mathcal{M}=\{c_m\}$ | Critic 插件链 |
| 输入 | $\bar{\mathbf{U}}$ | 上周期 warm-start 序列 |
| 输入 | $\Theta$ | $K,H,\Delta t,\lambda,\gamma,\Sigma,\ldots$ |
| 输出 | $\mathbf{u}_0^*$ | 本周期 `cmd_vel` |

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&\bar{\mathbf{U}} \leftarrow \mathrm{ShiftLeft}(\bar{\mathbf{U}});\quad
\bar{\mathbf{u}}_{H-1} \leftarrow \bar{\mathbf{u}}_{H-1} + \boldsymbol{\epsilon}
\qquad\text{（warm-start）} \\[6pt]
\textbf{2.}\;&\textbf{for } i = 1 \text{ to iteration\_count \textbf{ do}} \\[3pt]
&\quad \{\mathbf{U}^{(k)}\}_{k=1}^{K} \leftarrow \mathrm{SampleNoisedControls}(\bar{\mathbf{U}},\, \Sigma,\, \mathcal{U}_{limits})
\qquad\text{（§4.1）} \\[3pt]
&\quad \{\mathbf{X}^{(k)}\} \leftarrow \mathrm{BatchRollout}(\mathbf{x}_0,\, \{\mathbf{U}^{(k)}\},\, H,\, \Delta t)
\qquad\text{（Alg. 2; §3.2）} \\[3pt]
&\quad \{S^{(k)}\} \leftarrow \mathrm{ScoreTrajectories}(\{\mathbf{X}^{(k)}\},\, \{\mathbf{U}^{(k)}\},\, \mathcal{M})
\qquad\text{（Alg. 3; §4.1）} \\[3pt]
&\quad \bar{\mathbf{U}} \leftarrow \mathrm{SoftmaxAggregate}(\{\mathbf{U}^{(k)}\},\, \{S^{(k)}\},\, \lambda)
\qquad\text{（§4.2）} \\[6pt]
\textbf{3.}\;&\mathbf{u}_0^* \leftarrow \bar{\mathbf{u}}_0;\quad \bar{\mathbf{U}} \leftarrow \bar{\mathbf{U}}
\qquad\text{（保存 warm-start）} \\[6pt]
\textbf{4.}\;&\textbf{return } \mathbf{u}_0^*
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box algorithm-box-phase-b algorithm-box-subroutine">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 2</span>
    <span class="algorithm-box-title">BatchRollout 子程序</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{BatchRollout}(\mathbf{x}_0,\, \{\mathbf{U}^{(k)}\},\, H,\, \Delta t) \to \{\mathbf{X}^{(k)}\}$

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&\textbf{for } k = 1 \text{ to } K \textbf{ do} \\[3pt]
&\quad \mathbf{x}_0^{(k)} \leftarrow \mathbf{x}_0 \\[3pt]
&\quad \textbf{for } t = 0 \text{ to } H-1 \textbf{ do} \\[3pt]
&\quad\quad \mathbf{x}_{t+1}^{(k)} \leftarrow \Phi(\mathbf{x}_t^{(k)},\, \mathbf{u}_t^{(k)},\, \Delta t)
\qquad\text{（§3.2; DiffDrive / Omni）} \\[3pt]
&\quad \mathbf{X}^{(k)} \leftarrow \{\mathbf{x}_t^{(k)}\}_{t=0}^{H} \\[6pt]
\textbf{2.}\;&\textbf{return } \{\mathbf{X}^{(k)}\}
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box algorithm-box-phase-c algorithm-box-subroutine">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 3</span>
    <span class="algorithm-box-title">ScoreTrajectories 子程序</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{ScoreTrajectories}(\{\mathbf{X}^{(k)}\},\, \{\mathbf{U}^{(k)}\},\, \mathcal{M}) \to \{S^{(k)}\}$

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&\textbf{for } k = 1 \text{ to } K \textbf{ do} \\[3pt]
&\quad S^{(k)} \leftarrow 0 \\[3pt]
&\quad \textbf{for each } c_m \in \mathcal{M} \textbf{ do} \\[3pt]
&\quad\quad S^{(k)} \leftarrow S^{(k)} + w_m \cdot c_m(\mathbf{X}^{(k)},\, \mathbf{U}^{(k)},\, \mathcal{P},\, \mathcal{C}) \\[3pt]
&\quad S^{(k)} \mathrel{+}= \sum_{t=0}^{H-1} \tfrac{\gamma}{2}\,
{\mathbf{u}_t^{(k)}}^\top \Sigma^{-1} \bar{\mathbf{u}}_t
\qquad\text{（§4.1 正则）} \\[6pt]
\textbf{2.}\;&\textbf{return } \{S^{(k)}\}
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box-footer" markdown="1">

**Complexity** $O(K \cdot H \cdot M)$（$M=|\mathcal{M}|$）；默认 $2000\times56\approx 1.1\times10^5$ 步状态推进/周期，桌面 CPU **5–20 ms**（Nav2 xsimd 向量化）。Autonomy：`controller.lua` 已配置；插件 ⏳。

</div>

</div>

### 5.2 实现要点

- **与 DWB / NMPC**：DWB 优化**单步** $(v,\omega)$ 并 `argmin`；MPPI 优化**序列** $\mathbf{U}$ 并 softmax 加权（非 argmin）。NMPC 显式解 OCP（[§15 MPC](15_mpc_controller.md)）；MPPI 无求解器，靠 $K$ 覆盖搜索空间。
- **Critics**：Nav2 默认 Constraint / Cost / PathAlign / PathFollow / PathAngle / Goal / GoalAngle / PreferForward；`threshold_to_consider` 控制空间激活半径，应与 $H\Delta t$ 协调。详式与权重见 [configuring-mppic](https://docs.nav2.org/configuration/packages/configuring-mppic.html)。
- **调参方向**：贴障增大 CostCritic 权重；不跟路径增大 PathAlign；抖动增大 `gamma` 或减小 `temperature`；超时减 `batch_size` 或 `time_steps`。$f_{ctrl}=1/\mathrm{model\_dt}$。

---

## 6. 参考文献

1. Theodorou, E., Buchli, J., & Schaal, S. (2010). *Learning Policy Improvements with Path Integrals*. ICML / JMLR 11. [JMLR](https://jmlr.org/papers/v11/theodorou10a.html)
2. Williams, G., et al. (2017). *Information Theoretic MPC for Model-Based Reinforcement Learning*. ICRA. [PDF](https://homes.cs.washington.edu/~bboots/files/InformationTheoreticMPC.pdf)
3. Williams, G., Aldrich, A., Goldfain, B., & Theodorou, E. A. (2018). *Model Predictive Path Integral Control*. IROS.
4. Macenski, S., et al. (2020). *The Marathon 2: A Navigation System*. IROS. [DOI](https://doi.org/10.1109/IROS45743.2020.9341207)
5. Nav2 MPPI：[configuring-mppic](https://docs.nav2.org/configuration/packages/configuring-mppic.html) · [nav2_mppi_controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller)
