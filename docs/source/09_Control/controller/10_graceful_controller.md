(graceful-controller)=
# 10. Graceful Controller

> 归属 [§5.2 Graceful](../05_controller_algorithms.md#52-graceful-controller) · Autonomy ⏳ 配置预留，插件未实现
>
> **Graceful Controller**（Nav2 `nav2_graceful_controller`）实现 Park & Kuipers (ICRA 2011) **平滑控制律**：在自我中心极坐标下以奇异摄动分解快慢子系统，对路径上 **motion target** 位姿闭式输出 $(v,\omega)$；Nav2 叠加曲率降速、近目标减速与可选轨迹仿真碰撞检测。

---

## 1. 背景

局部路径跟踪除到达目标外，还需**运动品质**——有界速度、加速度与直观轨迹。经典 Pure Pursuit / DWA 多为 reactive 单步决策，加速度与 jerk 可任意大，不适于载人平台。**Park & Kuipers (2011)** 在 unicycle 自我中心极坐标下，以 Lyapunov + 奇异摄动导出全局收敛、无奇异点的 pose-following 控制律；路径跟踪通过在全局路径上选取远距离 **motion target** 并滚动应用该律实现（ICRA §IV）。Nav2 工程封装曲率–速度规则 (ICRA 式 (15))、`slowdown_radius`、初始旋转与 costmap 轨迹仿真（详见 §3–§5）。

---

## 2. 问题

**任务.** 平面差速机器人沿全局路径 $\mathcal{P}$ 向局部目标前进，输出平滑、可预测的 $(v_x^{cmd},\,\omega_z^{cmd})$。

**输入 / 输出.** 位姿 $\mathbf{x}$、路径 $\mathcal{P}^{(B)}$（机器人系）、可选 costmap $\mathcal{C}$ → 本周期速度指令；内部选定 motion target 位姿 $T$（位置 + 航向）。

**在线形式.** 每周期在 $\mathcal{P}^{(B)}$ 上取 lookahead **motion target**，由 ICRA 式 (13)(14) 得曲率 $\kappa$，再经式 (15) 与 Nav2 界选取 $v$、$\omega=\kappa v$；若启用碰撞检测，对预测轨迹前向仿真并在 costmap 上拒绝不可行 target（与 DWB/TEB 优化式避障不同）。

---

## 3. 运动模型

以下给出 §4 控制律所需链条：**自我中心极坐标运动学 → 快慢子系统分解**（ICRA §II–§III）。Nav2 将论文 $\theta$ 记为 $\phi$（`k_phi` 对应 $k_1$，`k_delta` 对应 $k_2$）。

### 3.1 自我中心极坐标（ICRA 式 (1)）

观测者位于机器人，视线指向 motion target $T$，距离 $r$，目标相对视线方位 $\phi$，车身航向相对视线为 $\delta$（ICRA Fig. 1；Nav2 `EgocentricPolarCoordinates`）。

**极坐标运动学**（ICRA 式 (1)）：

$$
\begin{pmatrix} \dot{r} \\ \dot{\phi} \\ \dot{\delta} \end{pmatrix}
=
\begin{pmatrix}
-v\cos\delta \\
\dfrac{v}{r}\sin\delta \\
\dfrac{v}{r}\sin\delta + \omega
\end{pmatrix}.
$$

- **$r,\phi,\delta$**：误差坐标；控制问题为将 $(r,\phi,\delta)^\top$ 驱动至原点（到达 target 位姿）。
- **$(v,\omega)$**：线速度、角速度；$v>0$ 时前进（倒车时 Nav2 翻转视线与航向）。

### 3.2 快慢子系统（ICRA 式 (2)–(3)）

将式 (1) 分为**慢子系统**（位置 $(r,\phi)$）与**快子系统**（转向 $\delta$）：

$$
\begin{pmatrix} \dot{r} \\ \dot{\phi} \end{pmatrix}
=
\begin{pmatrix}
-v\cos\delta \\
\dfrac{v}{r}\sin\delta
\end{pmatrix},
\qquad
\dot{\delta} = \dfrac{v}{r}\sin\delta + \omega.
$$

- **含义**：$\omega$ 仅直接作用于 $\delta$；通过虚拟航向 $\delta_{ref}$ 间接驱动 $(r,\phi)$。奇异摄动要求 $k_2\gg 1$ 使 $\delta$ 快速跟踪 $\delta_{ref}$，慢流形由式 (5) 刻画（ICRA Fig. 2）。

---

## 4. 数学问题定义

每周期对 motion target $T$ 在极坐标 $(r,\phi,\delta)$ 上应用**闭式反馈律**（非代价优化）：§4.1 给出 $\omega$ 与路径曲率 $\kappa$；§4.2 选取 $v$ 并施加 Nav2 工程约束。

### 4.1 平滑控制律（ICRA 式 (5)(13)(14)）

**慢流形 / 参考航向**（ICRA 式 (5)）：

$$
\delta_{ref} = \arctan(-k_1 \phi).
$$

- **$k_1>0$**（Nav2 `k_phi`）：$\dot\phi/\dot r$ 速率比；$k_1=0$ 退化为纯 waypoint 跟踪，$k_1$ 大则优先对齐目标航向（pose-following）。

**角速度控制律**（ICRA 式 (13)）：

$$
\omega = -\frac{v}{r}\Big[
k_2\bigl(\delta - \arctan(-k_1\phi)\bigr)
+ \Bigl(1 + \frac{k_1}{1+(k_1\phi)^2}\Bigr)\sin\delta
\Big].
$$

- **$k_2>0$**（Nav2 `k_delta`）：快子系统增益；$z=\delta-\delta_{ref}$ 指数收敛（ICRA 式 (11)–(12)）。

**路径曲率**（ICRA 式 (14)）：

$$
\kappa(r,\phi,\delta) = -\frac{1}{r}\Big[
k_2\bigl(\delta - \arctan(-k_1\phi)\bigr)
+ \Bigl(1 + \frac{k_1}{1+(k_1\phi)^2}\Bigr)\sin\delta
\Big],
\qquad \omega = \kappa\, v.
$$

- **含义**：$\kappa$ 与 $v$ 无关，轨迹形状由反馈状态唯一确定；$r\to 0$ 时需 §4.2 令 $v\to 0$ 以保证全局渐近稳定（parking 情形）。

### 4.2 线速度选取与约束

**曲率降速**（ICRA 式 (15)，路径跟踪）：

$$
v(\kappa) = \frac{v_{\max}}{1 + \beta\,|\kappa|^{\lambda}},
\qquad \beta>0,\;\lambda>1.
$$

- **含义**：$|\kappa|\to\infty$ 时 $v\to 0$，$|\kappa|\to 0$ 时 $v\to v_{\max}$；Nav2 `beta`、`lambda` 对应 $\beta,\lambda$（ICRA Fig. 6）。

**Nav2 附加界**（`SmoothControlLaw::calculateRegularVelocity`）：

$$
v \leftarrow \min\!\Big(
v(\kappa),\;
v_{\max}\,\frac{r}{r_{slow}},\;
\sqrt{2\, r\, a_{dec}}
\Big),
\quad
v \in [v_{\min},\, v_{\max}].
$$

- **$r_{slow}$**：`slowdown_radius`，消除 $r\to 0$ 奇异性；**$a_{dec}$**：`deceleration_max`。
- **角速度界**：$\omega\leftarrow\mathrm{clamp}(\kappa v,\,-\omega_{\max},\,\omega_{\max})$；若 $\kappa\neq 0$ 则回算 $v=\omega/\kappa$ 以保持 $\omega=\kappa v$。

**Motion target.** 在 $\mathcal{P}^{(B)}$ 上距机器人弧长 $L_d\in[L_{min},L_{max}]$（Nav2 首选用 `max_lookahead` 插值点）；沿路径自远及近尝试 target，直至 `simulateTrajectory` 在 $\mathcal{C}$ 上无碰撞（可选 `initial_rotation` 原地对齐视线方向）。

---

## 5. 求解

§4 在单周期内按 **motion target → 极坐标 → $\kappa$ → $v$ → $(v,\omega)$** 闭式执行；算法 1–3 与 Nav2 主流程对应。

### 5.1 算法（数学描述）

<div class="algorithm-box-diagram">

<div class="algorithm-box algorithm-box-phase-a">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 1</span>
    <span class="algorithm-box-title">Graceful 局部控制器</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{Graceful}(\mathbf{x},\, \mathcal{P},\, \mathcal{C};\, \Theta) \mapsto (v^{cmd},\, \omega^{cmd})$

  </div>
  <div class="algorithm-box-io" markdown="1">

| 方向 | 符号 | 说明 |
|------|------|------|
| 输入 | $\mathbf{x}$ | 位姿（机器人系原点） |
| 输入 | $\mathcal{P}^{(B)}$ | 变换后的全局路径 |
| 输入 | $\mathcal{C}$ | 局部 costmap（可选碰撞仿真） |
| 输入 | $\Theta$ | $k_1,k_2,\beta,\lambda,\,L_{max},\,r_{slow},\,\ldots$ |
| 输出 | $(v^{cmd},\,\omega^{cmd})$ | 本周期 `cmd_vel` |

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&\mathcal{P}^{(B)} \leftarrow \mathrm{TransformToRobotFrame}(\mathcal{P},\, \mathbf{x})
\qquad\text{（Nav2 base\_link）} \\[6pt]
\textbf{2.}\;&\text{若 XY 目标已到达：}\ (v,\omega) \leftarrow \mathrm{RotateInPlace}(\phi_{goal});\ \textbf{return}
\qquad\text{（可选碰撞检查）} \\[6pt]
\textbf{3.}\;&T \leftarrow \mathrm{GetLookAheadPoint}(\mathcal{P}^{(B)},\, L_{max})
\qquad\text{（motion target）} \\[6pt]
\textbf{4.}\;&\textbf{for } T' \text{ 沿路径由远及近：} \\[3pt]
&\quad \text{若 } \mathrm{ValidateTargetPose}(T',\, \mathcal{C}) \text{ 成功：} \\[3pt]
&\quad\quad (v^{cmd},\,\omega^{cmd}) \leftarrow \mathrm{SmoothControlLaw}(T')
\qquad\text{（Alg. 2）} \\[3pt]
&\quad\quad \textbf{return } (v^{cmd},\,\omega^{cmd}) \\[6pt]
\textbf{5.}\;&\textbf{throw } \text{NoValidControl}
\qquad\text{（无可行 target）}
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box algorithm-box-phase-b algorithm-box-subroutine">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 2</span>
    <span class="algorithm-box-title">SmoothControlLaw 子程序</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{SmoothControlLaw}(T) \to (v,\,\omega)$

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&(r,\phi,\delta) \leftarrow \mathrm{EgocentricPolarCoords}(T,\, \mathbf{x})
\qquad\text{（§3.1）} \\[6pt]
\textbf{2.}\;&\kappa \leftarrow -\tfrac{1}{r}\big[k_2(\delta-\arctan(-k_1\phi)) + (1+\tfrac{k_1}{1+(k_1\phi)^2})\sin\delta\big]
\qquad\text{（ICRA 式 (14)）} \\[6pt]
\textbf{3.}\;&v \leftarrow v_{\max}/(1+\beta|\kappa|^{\lambda})
\qquad\text{（ICRA 式 (15)）} \\[3pt]
&\quad v \leftarrow \min(v,\, v_{\max}\, r/r_{slow},\, \sqrt{2 r a_{dec}});\quad
v \leftarrow \mathrm{clamp}(v,\, v_{\min},\, v_{\max})
\qquad\text{（§4.2）} \\[6pt]
\textbf{4.}\;&\omega \leftarrow \mathrm{clamp}(\kappa v,\,-\omega_{\max},\,\omega_{\max});\quad
v \leftarrow \omega/\kappa \text{ if } \kappa\neq 0
\qquad\text{（保持 } \omega=\kappa v\text{）} \\[6pt]
\textbf{5.}\;&\textbf{return } (v,\,\omega)
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box algorithm-box-phase-c algorithm-box-subroutine">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 3</span>
    <span class="algorithm-box-title">ValidateTargetPose 子程序</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{ValidateTargetPose}(T,\, \mathcal{C}) \to \{\text{ok},\, (v,\,\omega)\}$

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&\text{可选：}\ \mathrm{RotateInPlace}(\arctan(y_T/x_T))
\qquad\text{（`initial\_rotation`）} \\[6pt]
\textbf{2.}\;&\text{以 } \Delta t \approx \text{resolution}/v_{\max} \text{ 前向积分：} \\[3pt]
&\quad \mathbf{x}_{k+1} \leftarrow \mathrm{IntegrateUnicycle}(\mathbf{x}_k,\, v_k,\, \omega_k,\, \Delta t)
\qquad\text{（Alg. 2 逐步）} \\[6pt]
\textbf{3.}\;&\text{若任一步 footprint 在 } \mathcal{C} \text{ 上代价超限：}\ \textbf{return fail} \\[6pt]
\textbf{4.}\;&\text{否则取首步 } (v_0,\,\omega_0) \text{ 为输出；}\ \textbf{return ok}
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box-footer" markdown="1">

**Complexity** $O(n+m)$（$n=|\mathcal{P}|$ target 搜索，$m$ 仿真步数）；典型 **&lt; 1 ms** / cycle（Nav2）。Autonomy：`controller.lua` 已预留 `graceful_controller` 段；插件 ⏳。

</div>

</div>

### 5.2 实现要点

- **与 RPP / DWB 关系**：Graceful 为 Lyapunov 闭式律 + motion target，非 PP 几何 $\kappa=2y/L^2$，亦非 DWB 速度采样；平滑性来自快慢子系统分离与式 (15) 曲率降速。
- **调参方向**：切弯穿墙启用碰撞仿真、减小 `max_lookahead`；近目标 overshoot 增大 `slowdown_radius`；起步摆头启用 `initial_rotation`。Nav2 参数见 [configuring-graceful-motion-controller](https://docs.nav2.org/configuration/packages/configuring-graceful-motion-controller.html)。

---

## 6. 参考文献

1. Park, J. J., & Kuipers, B. (2011). *A Smooth Control Law for Graceful Motion of Differential Wheeled Mobile Robots in 2D Environment*. IEEE ICRA. [DOI:10.1109/ICRA.2011.5980167](https://doi.org/10.1109/ICRA.2011.5980167)
2. Park, J. J. (2016). *Graceful Navigation for Mobile Robots in Dynamic and Uncertain Environments*. PhD thesis, Univ. of Michigan.
3. Nav2 Graceful Controller：[configuring-graceful-motion-controller](https://docs.nav2.org/configuration/packages/configuring-graceful-motion-controller.html) · [源码](https://github.com/ros-navigation/navigation2/tree/main/nav2_graceful_controller)
