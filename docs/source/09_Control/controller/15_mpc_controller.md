(mpc-controller)=
# 15. MPC Controller

> 归属 [§5.7 MPC](../05_controller_algorithms.md#57-模型预测控制mpc) · Autonomy ❌ 未实现
>
> **Model Predictive Control**（MPC）在离散预测时域 $H$ 上，对运动学模型前向滚动并**求解约束最优控制序列**；每周期仅执行首步 $\mathbf{u}_0^*$，再滚动重解。与 §11 **MPPI**（采样 + softmax）同属预测控制，但 MPC 通常依赖 QP/NLP 求解器而非路径积分采样。

---

## 1. 背景

局部控制需在约束下跟踪参考轨迹并避障。**模型预测控制**（Rawlings & Mayne, 2009）将有限时域最优控制问题在线反复求解：用模型预测未来 $H$ 步，最小化跟踪/控制代价并满足输入与状态约束，执行 $\mathbf{u}_0^*$ 后移位 warm-start。**NMPC**（非线性 MPC）直接嵌入差速 / car-like 非线性运动学，广泛用于研究型移动机器人平台；Nav2 默认栈以 MPPI（随机 MPC）替代显式 NLP 求解。本文给出经典 NMPC 数学形式（详见 §3–§5）；路径积分变体见 [§11 MPPI](11_mppi_controller.md)。

---

## 2. 问题

**任务.** 平面移动机器人在预测时域 $H\Delta t$ 内跟踪参考 $\mathbf{x}_k^{ref}$（由全局路径 $\mathcal{P}$ 生成），满足输入界与障碍约束，输出本周期控制 $\mathbf{u}_0^*$。

**输入 / 输出.** 位姿 $\mathbf{x}_0$、当前速度、路径 $\mathcal{P}$、障碍 / costmap $\mathcal{C}$ → $(v_x^{cmd},\,\omega_z^{cmd})$（或 car-like $[v,\,\phi]^\top$）。

**在线形式.** **滚动时域**（receding horizon）：每周期解一次 $H$ 步 OCP，执行 $\mathbf{u}_0^*$；下一周期以新状态 $\mathbf{x}_0$ 与移位后的控制序列 warm-start 重解（与 MPPI 单周期 batch 采样不同）。

---

## 3. 运动模型

以下给出 §4 OCP 的**离散预测模型**：连续差速运动学 → 欧拉离散化 → 滚动状态 $\mathbf{x}_k$。

### 3.1 差速连续运动学

世界系 $\{W\}$ 下 $\mathbf{x}=[x,y,\theta]^\top$，控制 $\mathbf{u}=[v,\,\omega]^\top$：

$$
\dot{x}=v\cos\theta,\quad \dot{y}=v\sin\theta,\quad \dot{\theta}=\omega.
$$

- **含义**：预测模型；$v,\omega$ 为决策变量序列 $\mathbf{u}_0,\ldots,\mathbf{u}_{H-1}$ 的分段恒定值。

### 3.2 离散预测模型

采样周期 $\Delta t$，欧拉离散（$k=0,\ldots,H-1$）：

$$
\begin{aligned}
x_{k+1} &= x_k + v_k\cos\theta_k\,\Delta t, \\
y_{k+1} &= y_k + v_k\sin\theta_k\,\Delta t, \\
\theta_{k+1} &= \theta_k + \omega_k\,\Delta t.
\end{aligned}
$$

紧凑记法 $\mathbf{x}_{k+1}=f(\mathbf{x}_k,\mathbf{u}_k)$，$\mathbf{u}_k=[v_k,\,\omega_k]^\top$。

- **$\mathbf{x}_0$**：当前测量位姿；**$H$**：预测步数；**$N=H$** 步 rollout 供 §4.1 代价累加。
- **Car-like**：可换为 bicycle 离散模型；NMPC 文献常用 SQP / 多重 shooting 处理非线性 $f$。

---

## 4. 数学问题定义

每周期在决策变量 $\mathbf{U}=\{\mathbf{u}_0,\ldots,\mathbf{u}_{H-1}\}$ 上求解**有限时域最优控制问题**（OCP）：§4.1 动力学与代价；§4.2 硬约束。

### 4.1 有限时域 OCP

$$
\mathbf{U}^* = \underset{\mathbf{U}}{\arg\min}\;
J(\mathbf{X},\mathbf{U})
=
\sum_{k=0}^{H-1}
\Big(
\|\mathbf{x}_k-\mathbf{x}_k^{ref}\|_Q^2
+ \|\mathbf{u}_k\|_R^2
\Big)
+ \|\mathbf{x}_H-\mathbf{x}_H^{ref}\|_{Q_f}^2,
$$

$$
\text{s.t.}\quad
\mathbf{x}_{k+1}=f(\mathbf{x}_k,\mathbf{u}_k),\quad k=0,\ldots,H-1,\quad
\mathbf{x}_0=\mathbf{x}_{meas}.
$$

- **$\mathbf{x}_k^{ref}$**：由 $\mathcal{P}$ 在时域上采样的参考状态；**$Q,R,Q_f$**：跟踪 / 控制 / 终端权重。
- **含义**：二次跟踪代价 + 控制正则；非线性 $f$ 使问题为 **NMPC**（非凸 NLP），非 LQR 闭式解。

### 4.2 约束（硬约束）

$$
\mathbf{u}_{min} \le \mathbf{u}_k \le \mathbf{u}_{max},\quad k=0,\ldots,H-1,
$$

$$
g(\mathbf{x}_k) \ge 0 \quad \text{（障碍 / 道路边界）}.
$$

- **输入界**：速度、角速度（及 car-like 的 $\phi$）上下界。
- **障碍**：$g$ 可为 signed distance 或 costmap 罚的**硬形式**；实践亦常用**软约束**（罚函数）以保 QP 可行性。
- **与 MPPI**：MPPI 将约束并入轨迹代价 $S(\mathbf{U})$ 采样评估；MPC 显式写入 `s.t.` 或由 SQP 罚函数处理。

---

## 5. 求解

§4 OCP 每周期经 **predict → optimize → shift** 执行；算法 1–2 给出主流程与 NLP 子程序（RTI / SQP 抽象）。

### 5.1 算法（数学描述）

<div class="algorithm-box-diagram">

<div class="algorithm-box algorithm-box-phase-a">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 1</span>
    <span class="algorithm-box-title">MPC 局部控制器</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{MPC}(\mathbf{x}_{meas},\, \mathcal{P},\, \mathcal{C};\, H,\, \Delta t,\, \Theta) \mapsto \mathbf{u}_0^*$

  </div>
  <div class="algorithm-box-io" markdown="1">

| 方向 | 符号 | 说明 |
|------|------|------|
| 输入 | $\mathbf{x}_{meas}$ | 当前位姿（及速度） |
| 输入 | $\mathcal{P}$ | 全局路径 → $\mathbf{x}_k^{ref}$ |
| 输入 | $\mathcal{C}$ | 障碍 / 约束 |
| 输入 | $H,\,\Delta t$ | 预测步数、采样周期 |
| 输入 | $\Theta$ | $Q,R,Q_f$，$\mathbf{u}_{min/max}$，… |
| 输入 | $\mathbf{U}_{init}$ | 上周期移位 warm-start（可选） |
| 输出 | $\mathbf{u}_0^*$ | 本周期控制 |

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&\{\mathbf{x}_k^{ref}\}_{k=0}^{H} \leftarrow \mathrm{SampleRef}(\mathcal{P},\, H,\, \Delta t)
\qquad\text{（路径参考）} \\[6pt]
\textbf{2.}\;&\textbf{if } \mathbf{U}_{init}=\emptyset \textbf{ then }
\mathbf{U} \leftarrow \mathrm{InitControl}(H)
\;\textbf{ else }\;
\mathbf{U} \leftarrow \mathrm{Shift}(\mathbf{U}_{init})
\qquad\text{（warm-start）} \\[6pt]
\textbf{3.}\;&\mathbf{U}^* \leftarrow \mathrm{SolveOCP}(\mathbf{x}_{meas},\, \mathbf{U},\, \{\mathbf{x}_k^{ref}\},\, \mathcal{C})
\qquad\text{（Alg. 2; §4.1–§4.2）} \\[6pt]
\textbf{4.}\;&\mathbf{u}_0^* \leftarrow \mathbf{u}_0 \text{ from } \mathbf{U}^* \\[6pt]
\textbf{5.}\;&\mathbf{U}_{init} \leftarrow \mathrm{Shift}(\mathbf{U}^*)
\qquad\text{（下一周期）} \\[6pt]
\textbf{6.}\;&\textbf{return } \mathbf{u}_0^*
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box algorithm-box-phase-b algorithm-box-subroutine">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 2</span>
    <span class="algorithm-box-title">SolveOCP 子程序</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{SolveOCP}(\mathbf{x}_0,\, \mathbf{U},\, \{\mathbf{x}_k^{ref}\},\, \mathcal{C}) \to \mathbf{U}^*$

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&\textbf{repeat } \text{（SQP / RTI 迭代）} \\[3pt]
&\quad \mathbf{X} \leftarrow \mathrm{Predict}(\mathbf{x}_0,\, \mathbf{U},\, f,\, H)
\qquad\text{（§3.2）} \\[3pt]
&\quad \text{线性化 } f \text{ 得 } A_k,\, B_k;\;
\text{组装 QP/NLP 子问题（§4.1–§4.2）} \\[3pt]
&\quad \Delta\mathbf{U} \leftarrow \mathrm{LinearSolve}(\ldots) \\[3pt]
&\quad \mathbf{U} \leftarrow \mathbf{U} + \Delta\mathbf{U} \\[6pt]
\textbf{2.}\;&\mathbf{U}^* \leftarrow \mathbf{U};\quad \textbf{return } \mathbf{U}^*
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box-footer" markdown="1">

**Complexity** $O(H^3)$–$O(H \cdot n_{iter})$（QP/NLP 规模随 $H$ 增长）；典型 **10–50 ms** / cycle（视 $H$ 与求解器）。Autonomy 未集成；**采样型 MPC** 见 [§11 MPPI](11_mppi_controller.md)。

</div>

</div>

### 5.2 实现要点

- **NMPC vs MPPI**：NMPC 显式优化 $\mathbf{U}^*$（SQP/RTI/acados 等）；MPPI 用 $K$ 条随机轨迹 + softmax，无需求解器，见 [06_survey §6](../06_survey.md)。
- **可行性**：障碍硬约束易导致 infeasible；常用软约束、slack 或缩短 $H$。
- **实时性**：$H$ 与 $\Delta t$ 权衡预测长度与计算量；移位 warm-start 为关键。

---

## 6. 参考文献

1. Rawlings, J. B., & Mayne, D. Q. (2009). *Model Predictive Control: Theory and Design*. Nob Hill Publishing.
2. Borrelli, F., Bemporad, A., & Morari, M. (2017). *Predictive Control for Linear and Hybrid Systems*. Cambridge.
3. Williams, G., et al. (2017). *Information Theoretic MPC for Model-Based Reinforcement Learning*. ICRA. [PDF](https://homes.cs.washington.edu/~bboots/files/InformationTheoreticMPC.pdf)（→ MPPI，见 §11）
4. Grüne, L., & Pannek, J. (2017). *Nonlinear Model Predictive Control*. Springer.
