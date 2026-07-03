(rpp-controller)=
# 12. Regulated Pure Pursuit (RPP) Controller

> 归属 [§5.4 RPP](../05_controller_algorithms.md#54-regulated-pure-pursuitrpp) · Autonomy ⏳ 工具函数就绪，插件未实现
>
> **Regulated Pure Pursuit**（Nav2 `nav2_regulated_pure_pursuit_controller`）在 Coulter (1992) **Pure Pursuit** 几何跟踪上，叠加曲率 / 障碍 / 目标三项**线速度调节**；每周期由 lookahead 曲率得 $\omega$，输出 $(v_x,\omega_z)$。

---

## 1. 背景

局部路径跟踪需在低算力下稳定跟随全局路径。**Pure Pursuit**（Coulter, 1992）在路径上取距机器人 $L_d$ 的 lookahead 点，以圆弧几何关系 $\kappa=2y_l/L_d^2$ 将横向偏差映射为角速度。**Regulated Pure Pursuit**（Nav2 默认）在 PP 输出前对 $v_x$ 取 $\min$ 调节，避免急弯、近障与近目标时超速（详见 §3–§5）。

---

## 2. 问题

**任务.** 平面差速机器人沿全局路径 $\mathcal{P}$ 向局部目标前进，在 costmap $\mathcal{C}$ 约束下输出本周期速度指令。

**输入 / 输出.** 位姿 $\mathbf{x}$、当前 $v_x$、路径 $\mathcal{P}$、代价地图 $\mathcal{C}$（障碍调节）→ $(v_x^{cmd},\,\omega_z^{cmd})$。

**在线形式.** 每周期几何闭式计算（无轨迹采样/优化）；路径变换到**机器人坐标系**后求 lookahead，再得 $\kappa$ 与调节后 $v_x$（Coulter 1992；Nav2 RPP）。

---

## 3. 运动模型

以下给出 §4 所需链条：**差速运动学 → 机器人系路径几何 → lookahead 点**。

### 3.1 差速连续运动学

**坐标系与状态.** 世界系 $\{W\}$ 下 $\mathbf{x}=[x,y,\theta]^\top$；控制 $(v_x,\omega_z)$（Coulter / Nav2 记号）：

$$
\dot{x}=v_x\cos\theta,\quad \dot{y}=v_x\sin\theta,\quad \dot{\theta}=\omega_z.
$$

- **含义**：Pure Pursuit 假设 $v_x$ 沿车身 $x$ 轴；$\omega_z=\kappa v_x$ 由 §4.2 几何给出，非独立优化变量。

### 3.2 机器人坐标系与路径

将 $\mathcal{P}=\{p_j\}_{j=0}^{N}$ 经 TF 变换到**机器人系** $\{B\}$：机器人位于原点、航向沿 $+x$。

$$
p_j^{(B)} = R(-\theta)\,\bigl(p_j^{(W)} - [x,\,y]^\top\bigr),\qquad
R(\alpha)=\begin{bmatrix}\cos\alpha&-\sin\alpha\\ \sin\alpha&\cos\alpha\end{bmatrix}.
$$

- **$p_{la}^{(B)}=(x_l,y_l)$**：§4.1 在 $\mathcal{P}^{(B)}$ 上距原点弧长 $L_d$ 的 lookahead 点（§3.3 弧长累积 + 线性插值）。
- **含义**：后续 $\kappa$ 在 $\{B\}$ 下计算；Autonomy `GetLookAheadPoint()` 实现该步骤。

### 3.3 弧长插值（`GetLookAheadPoint`）

沿 $\mathcal{P}^{(B)}$ 累积段长 $\Delta s_i=\|p_i-p_{i-1}\|_2$，$S_k=\sum_{i=1}^k\Delta s_i$。找最小 $k$ 使 $S_{k-1}<L_d\leq S_{k-1}+\Delta s_k$；$\delta=L_d-S_{k-1}$，$\alpha=\delta/\Delta s_k$，则 $p_{la}=p_{k-1}+\alpha(p_k-p_{k-1})$（`LinearInterpolation`）。

### 3.4 圆-线段交点（`CircleSegmentIntersection`）

圆 $x^2+y^2=r^2$ 与线段 $P_1P_2$ 参数化 $x=x_1+t\,dx$，$y=y_1+t\,dy$，$t\in[0,1]$。令 $D=x_1y_2-x_2y_1$，$dr^2=dx^2+dy^2$，判别式 $\Delta=r^2 dr^2-D^2$。Autonomy 用 $\mathrm{sign}(d_2^2-d_1^2)$ 选取落在线段上的交点 $(x^*,y^*)$，供 PP 几何辅助。

---

## 4. 数学问题定义

每周期依次确定 lookahead 距离 $L_d$、曲率 $\kappa$、调节后线速度 $v_x$，再输出 $\omega_z=\kappa v_x$。

### 4.1 Lookahead 距离

Nav2 动态前瞻（速度缩放）：

$$
L_d = \mathrm{clamp}\!\big(\alpha\,|v_x| + L_{base},\; L_{min},\; L_{max}\big).
$$

- **$L_{base},L_{min},L_{max}$**：Nav2 `lookahead_dist` 等；$\alpha$ 为速度缩放系数（`use_velocity_scaled_lookahead`）。
- **$p_{la}$**：在 $\mathcal{P}^{(B)}$ 上沿弧长 $L_d$ 插值得 $(x_l,y_l)$；路径总长 $<L_d$ 时取末点或外推（`interpolate_after_goal`）。

### 4.2 Pure Pursuit 曲率（Coulter 1992）

机器人系下 lookahead $P=(x_l,y_l)$，沿过 $O$ 与 $P$ 且与 $x$ 轴相切的圆弧跟踪：

$$
\kappa = \frac{2 y_l}{x_l^2 + y_l^2}
\approx \frac{2 y_l}{L_d^2},\qquad L_d=\sqrt{x_l^2+y_l^2}.
$$

**控制律**：

$$
\omega_z = \kappa \cdot v_x.
$$

- **$\kappa$**：路径曲率；$y_l>0$ 左转，$y_l<0$ 右转；$|y_l|\to 0$ 时 $\kappa\to 0$（直线）。
- **含义**：几何跟踪，非代价优化；精确式用分母 $x_l^2+y_l^2$，实现常用 $L_d^2$ 近似（§4.2）。

### 4.3 Regulated 线速度（Nav2）

在 §4.2 之前对期望线速度取多项 $\min$ 调节：

$$
v_x \leftarrow \min\!\left(v_{desired},\;
\frac{\omega_{max}}{|\kappa|+\epsilon},\;
\alpha_{obs}\, d_{obs},\;
\alpha_{goal}\, d_{goal}\right).
$$

- **曲率项** $\omega_{max}/|\kappa|$：急弯降速，等价于限制最小转弯半径 $R_{min}\approx v_x/|\omega_z|$。
- **$d_{obs}$**：沿路径或 costmap 前方障碍距离；**$d_{goal}$**：距局部目标距离。
- **含义**：三项调节将 PP 几何输出限制在可执行、安全速度内；$\omega_z$ 仍由调节后 $v_x$ 与 §4.2 $\kappa$ 相乘。

---

## 5. 求解

§4 在单周期内按 **lookahead → 曲率 → 调节 → 输出** 执行；算法 1–3 与之逐步对应。

### 5.1 算法（数学描述）

<div class="algorithm-box-diagram">

<div class="algorithm-box algorithm-box-phase-a">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 1</span>
    <span class="algorithm-box-title">RPP 局部控制器</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{RPP}(\mathbf{x}, v_x, \mathcal{P}, \mathcal{C};\, \Theta) \mapsto (v_x^{cmd},\, \omega_z^{cmd})$

  </div>
  <div class="algorithm-box-io" markdown="1">

| 方向 | 符号 | 说明 |
|------|------|------|
| 输入 | $\mathbf{x}$ | 位姿 $(x,y,\theta)$ |
| 输入 | $v_x$ | 当前/期望线速度 |
| 输入 | $\mathcal{P}$ | 全局参考路径 |
| 输入 | $\mathcal{C}$ | 局部 costmap（障碍调节） |
| 输入 | $\Theta$ | $L_{base},L_{min},L_{max},\,\omega_{max},\,\ldots$ |
| 输出 | $(v_x^{cmd},\,\omega_z^{cmd})$ | 本周期 `cmd_vel` |

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&\mathcal{P}^{(B)} \leftarrow \mathrm{TransformToRobotFrame}(\mathcal{P},\, \mathbf{x})
\qquad\text{（§3.2）} \\[6pt]
\textbf{2.}\;&L_d \leftarrow \mathrm{Clamp}(\alpha|v_x|+L_{base},\, L_{min},\, L_{max})
\qquad\text{（§4.1）} \\[6pt]
\textbf{3.}\;&p_{la} \leftarrow \mathrm{GetLookAheadPoint}(\mathcal{P}^{(B)},\, L_d)
\qquad\text{（Alg. 2; 03\_math §3.3）} \\[6pt]
\textbf{4.}\;&\kappa \leftarrow 2 y_{la} / L_d^2
\qquad\text{（§4.2; 或 }2y/(x^2+y^2)\text{）} \\[6pt]
\textbf{5.}\;&v_x^{cmd} \leftarrow \mathrm{RegulateVelocity}(\kappa,\, d_{obs},\, d_{goal},\, \mathcal{C})
\qquad\text{（Alg. 3; §4.3）} \\[6pt]
\textbf{6.}\;&\omega_z^{cmd} \leftarrow \kappa \cdot v_x^{cmd}
\qquad\text{（§4.2）} \\[6pt]
\textbf{7.}\;&\textbf{return } (v_x^{cmd},\, \omega_z^{cmd})
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box algorithm-box-phase-b algorithm-box-subroutine">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 2</span>
    <span class="algorithm-box-title">GetLookAheadPoint 子程序</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{GetLookAheadPoint}(\mathcal{P}^{(B)}, L_d) \to p_{la}=(x_l,y_l)$

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&\text{沿 } \mathcal{P}^{(B)} \text{ 累积弧长 } S_k;\quad \text{找段 } k \text{ 使 } S_{k-1}<L_d\le S_k \\[6pt]
\textbf{2.}\;&\alpha \leftarrow (L_d-S_{k-1})/\Delta s_k;\quad
p_{la} \leftarrow p_{k-1}+\alpha(p_k-p_{k-1})
\qquad\text{（线性插值）} \\[6pt]
\textbf{3.}\;&\textbf{return } p_{la}
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box algorithm-box-phase-c algorithm-box-subroutine">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 3</span>
    <span class="algorithm-box-title">RegulateVelocity 子程序</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{RegulateVelocity}(\kappa, d_{obs}, d_{goal}, \mathcal{C}) \to v_x^{cmd}$

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&v_x^{cmd} \leftarrow v_{desired} \\[6pt]
\textbf{2.}\;&v_x^{cmd} \leftarrow \min\bigl(v_x^{cmd},\,\omega_{max}/(|\kappa|+\epsilon)\bigr)
\qquad\text{（曲率调节）} \\[3pt]
&\quad v_x^{cmd} \leftarrow \min\bigl(v_x^{cmd},\,\alpha_{obs}\, d_{obs}\bigr)
\qquad\text{（障碍调节）} \\[3pt]
&\quad v_x^{cmd} \leftarrow \min\bigl(v_x^{cmd},\,\alpha_{goal}\, d_{goal}\bigr)
\qquad\text{（目标调节）} \\[6pt]
\textbf{3.}\;&\textbf{return } v_x^{cmd}
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box-footer" markdown="1">

**Complexity** $O(n)$（$n=|\mathcal{P}|$ 路径点数）；典型 **&lt; 0.1 ms** / cycle。Autonomy：`GetLookAheadPoint`、`CircleSegmentIntersection`、`LinearInterpolation`（`controller_utils.*`）已实现 Step 2–4 几何；RPP 插件 ⏳。

</div>

</div>

### 5.2 实现要点

- **与 PP 关系**：Regulated PP = Pure Pursuit（§4.2）+ §4.3 线速度 $\min$ 调节；无 DWB/TEB 式采样。
- **调参方向**：窄道减小 $L_d$、启用 cost regulated；开阔地增大 $L_d$；振荡时增大 $L_{min}$。Nav2 参数见 [configuring-rpp-controller](https://navigation.ros.org/configuration/packages/configuring-regulated-pp-controller.html)。

---

## 6. 参考文献

1. Coulter, R. (1992). *Implementation of the Pure Pursuit Path Tracking Algorithm*. CMU-RI-TR-92-01. [CMU](https://www.ri.cmu.edu/publications/implementation-of-the-pure-pursuit-path-tracking-algorithm/)
2. Macenski, S., et al. (2020). *The Marathon 2: A Navigation System*. IEEE/RSJ IROS. [DOI](https://doi.org/10.1109/IROS45743.2020.9341207)
3. Nav2 RPP：[configuring-rpp-controller](https://navigation.ros.org/configuration/packages/configuring-regulated-pp-controller.html)
