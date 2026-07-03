(dwb-controller)=
# 13. DWB Controller

> 归属 [§5.5 DWB](../05_controller_algorithms.md#55-dwbdynamic-window-benchmark) · Autonomy ❌ 未实现
>
> **Dynamic Window Benchmark**（DWB）在 Fox (1997) **DWA** 之上，以 Lu (2014) 可插拔 **Critic** 框架工程化：每周期在 $\mathcal{V}_{legal}$ 内网格采样 $(v,\omega)$，rollout 短轨迹并加权选优，输出 $(v^*,\omega^*)$。

---

## 1. 背景

局部控制需在运动学约束与动态障碍下，每周期决定线速度与角速度。**Dynamic Window Approach**（Fox et al., 1997）在电机可达、动态窗口与可刹停交集内采样 $(v,\omega)$，前向仿真后按航向、 clearance、速度三项代价选优。**DWB**（Nav2 `nav2_dwb_controller`）将三项扩展为 Critic 插件链（详见 §3–§5）。

---

## 2. 问题

**任务.** 平面移动机器人沿参考路径 $\mathcal{P}$ 向局部目标前进，在 costmap $\mathcal{C}$ 下选取本周期速度指令。

**输入 / 输出.** 位姿 $\mathbf{x}$、当前速度 $(v_c,\omega_c)$、路径 $\mathcal{P}$、代价地图 $\mathcal{C}$、Critic 集 $\mathcal{M}$ → $(v^*,\omega^*)$（差速 `cmd_vel`；全向含 $v_y$）。

**在线形式.** 每周期独立求解离散速度对 $(v,\omega)$（非整条轨迹优化）；$(v_c,\omega_c)$ 经 §4.1 动态窗口 $\mathcal{V}_d$ 与上一周期耦合。

---

## 3. 运动模型

以下给出 §4.2 评价与算法 2 所需链条：**连续运动学 → 离散 rollout → 轨迹 $\tau$**（Fox 1997）。

### 3.1 差速连续运动学

差速平台以 $(v,\omega)$ 为控制量，作为 §3.2 rollout 与 §4.1 动态窗口的连续时间基准。

**坐标系与状态.** 世界系 $\{W\}$ 下：

$$
\mathbf{x}(t) = \begin{bmatrix} x(t) \\ y(t) \\ \theta(t) \end{bmatrix}
\in \mathbb{R}^2 \times S^1.
$$

- **$(x,y)$**：机器人参考点位置（通常为 base_link）。
- **$\theta$**：平面航向角。
- **$(v,\omega)$**：本周期候选线速度、角速度，分段恒定（不直接优化 $\dot{v},\dot{\omega}$）。

**连续运动学**（Fox 1997）：

$$
\dot{\mathbf{x}}(t)=
\begin{bmatrix}
\dot{x} \\ \dot{y} \\ \dot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
v\cos\theta \\ v\sin\theta \\ \omega
\end{bmatrix}.
$$

- **含义**：纯运动学模型，不含力/惯量；线/角加减速界在 §4.1 $\mathcal{V}_d$、$\mathcal{V}_a$ 施加。

**全向扩展**（Nav2 `Omni`）：增加侧向 $v_y$，$\dot{x}=v_x\cos\theta-v_y\sin\theta$，$\dot{y}=v_x\sin\theta+v_y\cos\theta$；§5 采样维数增加，结构不变。

### 3.2 离散 Rollout

给定候选 $(v,\omega)$，从当前 $\mathbf{x}$ 前向积分得仿真轨迹 $\tau$，供 §4.2 $C_{total}$ 评价（算法 2）。

**单步映射** $\mathbf{x}_{k+1}=\Phi(\mathbf{x}_k,v,\omega,\Delta t)$。**欧拉**（$|\omega|\Delta t$ 小）：

$$
\mathbf{x}_{k+1} = \mathbf{x}_k +
\begin{bmatrix} v\cos\theta_k \\ v\sin\theta_k \\ \omega \end{bmatrix}\Delta t.
$$

**圆弧**（$\omega\neq 0$，半径 $R=v/\omega$；Fox 1997，Nav2 可切换）：

$$
\begin{bmatrix} x_{k+1} \\ y_{k+1} \\ \theta_{k+1} \end{bmatrix}
= \begin{bmatrix} x_k \\ y_k \\ \theta_k \end{bmatrix}
+ \begin{bmatrix}
-\frac{v}{\omega}\sin\theta_k + \frac{v}{\omega}\sin(\theta_k+\omega\Delta t) \\
\frac{v}{\omega}\cos\theta_k - \frac{v}{\omega}\cos(\theta_k+\omega\Delta t) \\
\omega\Delta t
\end{bmatrix}.
$$

- **$\tau$**：迭代 $N=\lfloor T_{\mathrm{sim}}/\Delta t_{\mathrm{sim}}\rfloor$ 步，$\tau=\{\mathbf{x}_k\}_{k=0}^{N}$；$T_{\mathrm{sim}}$ 为**评价时域**，非全局路径长度。
- **$\mathbf{x}_N$**：终点位姿；$\theta_N$、$\tau$ 上 $d_{min}$ 等供 §4.2 Critic 读取。

---

## 4. 数学问题定义

先由 §4.1 构造可行速度集合 $\mathcal{V}_{legal}$，再在 §4.2 上对 rollout 轨迹最小化 $C_{total}$（Fox 1997 → Lu / Nav2 Critic 链）。

### 4.1 合法速度空间（硬约束）

$$
\begin{aligned}
\mathcal{V}_{legal} &= \mathcal{V}_s \cap \mathcal{V}_d \cap \mathcal{V}_a, \\[4pt]
\mathcal{V}_s &= [v_{\min},v_{\max}]\times[\omega_{\min},\omega_{\max}], \\[4pt]
\mathcal{V}_d &= \big\{v\in[v_c-\dot{v}_{dec}\,dt,\,v_c+\dot{v}_{acc}\,dt],\;
\omega\in[\omega_c-\dot{\omega}_{dec}\,dt,\,\omega_c+\dot{\omega}_{acc}\,dt]\big\}, \\[4pt]
\mathcal{V}_a &= \big\{v\le\sqrt{2d\,\dot{v}_{brake}},\;\omega\le\sqrt{2d_\theta\,\dot{\omega}_{brake}}\big\}.
\end{aligned}
$$

- **$\mathcal{V}_s$**：电机能力；**$\mathcal{V}_d$**：一周期内可达动态窗口；**$\mathcal{V}_a$**：当前速度下可安全刹停。
- **离散化**：$\{(v_i,\omega_j)\}=\mathrm{Grid}(\mathcal{V}_{legal},N_v,N_\omega)$，$K=N_vN_\omega$ 组候选（算法 1 步骤 1–5）。

### 4.2 速度决策主问题

$$
(v^*, \omega^*) = \arg\min_{(v,\omega) \in \mathcal{V}_{legal}} C_{total}\!\big(\tau(v,\omega)\big).
$$

**Critic 加权**（DWB / Nav2）：

$$
C_{total}(\tau, v) = \sum_{m=1}^{M} w_m \cdot c_m(\tau, v, \mathcal{P}, \mathcal{C}).
$$

- **$\tau(v,\omega)$**：§3.2 以 $(v,\omega)$ rollout 得 $\tau$（见 §3.2 终点 $\mathbf{x}_N$）。
- **$c_m$** / **$w_m$**：Critic 原始得分 / Nav2 `scale`；$c_m=+\infty$（碰撞）时算法 3 **短路截断**。

**Fox 1997 三项**（经典 DWA；批次归一化后以 $\alpha,\beta,\gamma$ 加权；DWB 以 Critic 插件实现，$w_m$ 对应 $\alpha,\beta,\gamma$）：

$$
C_{total} \approx \alpha\,\frac{\Delta\theta(\theta_N,\,\theta_{target})}{\pi}
+ \beta\,\frac{1}{d_{min}+\epsilon}
+ \gamma\,(v_{max}-v).
$$

- **航向**：$\Delta\theta$ 为 $\theta_N$ 与 $\theta_{target}$ 之差（归一化到 $[0,1]$）。
- **Clearance**：$d_{min}$ 为 $\tau$ 上最小障碍距离；碰撞时 $+\infty$。
- **速度**：$v_{max}-v$ 鼓励前进。

---

## 5. 求解

§4 在单周期内按 **A → B → C** 执行：**A** 构造 $\mathcal{V}_{legal}$ 并采样（§4.1）→ **B** rollout $\{\tau_i\}$（§3.2）→ **C** $\arg\min C_{total}$（§4.2）。算法 1–3 与之逐步对应。

### 5.1 算法（数学描述）

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
| 输入 | $\mathbf{x}$ | 位姿 $(x,y,\theta)$ |
| 输入 | $(v_c,\omega_c)$ | 当前线速度、角速度 |
| 输入 | $\mathcal{P}$ | 全局参考路径 |
| 输入 | $\mathcal{C}$ | 局部 costmap |
| 输入 | $\mathcal{M}=\{c_m\}_{m=1}^{M}$ | TrajectoryCritic 插件集 |
| 输入 | $\Theta$ | $N_v,N_\omega,T_{\mathrm{sim}},\Delta t_{\mathrm{sim}},\ldots$ |
| 输出 | $(v^*,\omega^*)$ | 本周期 `cmd_vel` |

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&\mathcal{V}_s \leftarrow [v_{\min}, v_{\max}] \times [\omega_{\min}, \omega_{\max}]
\qquad\text{（§4.1 $\mathcal{V}_s$）} \\[6pt]
\textbf{2.}\;&\mathcal{V}_d \leftarrow \mathrm{DynamicWindow}(v_c, \omega_c, \dot{v}_{acc/dec}, dt)
\qquad\text{（§4.1 $\mathcal{V}_d$）} \\[6pt]
\textbf{3.}\;&\mathcal{V}_a \leftarrow \mathrm{BrakingDist}(d, d_\theta, \dot{v}_{brake}, \dot{\omega}_{brake})
\qquad\text{（§4.1 $\mathcal{V}_a$）} \\[6pt]
\textbf{4.}\;&\mathcal{V}_{\mathrm{legal}} \leftarrow \mathcal{V}_s \cap \mathcal{V}_d \cap \mathcal{V}_a \\[6pt]
\textbf{5.}\;&\mathcal{C}_{\mathrm{vel}} \leftarrow \mathrm{GridSample}(\mathcal{V}_{\mathrm{legal}}, N_v, N_\omega)
\qquad\text{（§4.1; $K=N_v N_\omega$）} \\[6pt]
\textbf{6.}\;&\mathcal{T} \leftarrow \emptyset \\[6pt]
\textbf{7.}\;&\textbf{for each } (v_i, \omega_i) \in \mathcal{C}_{\mathrm{vel}} \textbf{ do} \\[3pt]
&\quad \tau_i \leftarrow \mathrm{Rollout}(\mathbf{x}, v_i, \omega_i, T_{\mathrm{sim}}, \Delta t_{\mathrm{sim}})
\qquad\text{（Alg. 2; §3.2）} \\[3pt]
&\quad \mathcal{T} \leftarrow \mathcal{T} \cup \{(v_i, \omega_i, \tau_i)\} \\[6pt]
\textbf{8.}\;&(v^*, \omega^*) \leftarrow \underset{(v_i,\omega_i,\tau_i) \in \mathcal{T}}{\arg\min}\;
\mathrm{Score}(\tau_i, v_i, \mathcal{P}, \mathcal{C}, \mathcal{M})
\qquad\text{（Alg. 3; §4.2）} \\[6pt]
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
\qquad\text{（§3.2 Euler / arc）} \\[3pt]
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

$\mathrm{Score}(\tau, v, \mathcal{P}, \mathcal{C}, \mathcal{M}) \to S \equiv C_{total}(\tau,v)$

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&S \leftarrow 0 \\[6pt]
\textbf{2.}\;&\textbf{for each } c_m \in \mathcal{M} \textbf{ do} \\[3pt]
&\quad s_m \leftarrow c_m.\mathrm{score}(\tau, v, \mathcal{P}, \mathcal{C}) \\[3pt]
&\quad \textbf{if } s_m = +\infty \textbf{ then return } +\infty
\qquad\text{（collision short-circuit）} \\[3pt]
&\quad S \leftarrow S + w_m \cdot s_m \\[6pt]
\textbf{3.}\;&\textbf{return } S
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box-footer" markdown="1">

**A**（步骤 1–5）$\to$ **B**（步骤 7 + Alg. 2）$\to$ **C**（步骤 8 + Alg. 3）。Nav2：`TrajectoryGenerator` + `TrajectoryCritic[]`；`short_circuit_trajectory_evaluation` 对应 Alg. 3。**Complexity** $O(K \cdot N \cdot M)$; typical **1–5 ms** / cycle.

</div>

</div>

### 5.2 离散决策与 Critic

- **局部最优**：$K=N_vN_\omega$ 有限网格，非连续 $\arg\min$；Oscillation 等跨周期 Critic 使目标依赖历史状态。
- **$c_m$ 实现**：障碍类沿 $\tau$ 采样 costmap / 足迹；路径/目标类多用 $\mathbf{x}_N$ 或前向点。详式与参数见 Lu (2014) 与 [Nav2 DWB](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html)。

---

## 6. 参考文献

1. Fox, D., Burgard, W., & Thrun, S. (1997). *The Dynamic Window Approach to Collision Avoidance*. IEEE RAM. [IEEE](https://ieeexplore.ieee.org/document/619666)
2. Lu, D. (2014). *Navigation and Control of Mobile Robots*. PhD thesis, CMU. [DOI](https://doi.org/10.7936/K77P8WHT)
3. Macenski, S., et al. (2020). *The Marathon 2: A Navigation System*. IEEE/RSJ IROS. [DOI](https://doi.org/10.1109/IROS45743.2020.9341207)
4. Nav2 源码：[nav2_dwb_controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_dwb_controller)
