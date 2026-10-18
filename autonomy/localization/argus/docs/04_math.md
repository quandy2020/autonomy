# 数学原理

> Argus 估计核：误差状态 ESIKF（点–面）+ 可选直接法 VIO；旁路含退化 / 门控、Scan Context 回环、SE(3) 位姿图与 BnB 重定位。公式对照 `StatesGroup`、`ImuProcess`、`EsikfStateFilter`、`VIOManager` 等实现；行内 `$…$`，独立块 `$$…$$`。

时序见 [03_sequence.md](03_sequence.md)；门控配置见 [05_frontend_fusion.md](05_frontend_fusion.md)。

---

## 1. 流形与李代数

**Rodrigues 指数 / 对数.** 姿态 $R\in\mathrm{SO}(3)$，旋转向量 $\boldsymbol{\phi}$，$\theta=\lVert\boldsymbol{\phi}\rVert$（`math::Exp` / `math::Log`）：

$$
\begin{aligned}
\mathrm{Exp}(\boldsymbol{\phi})
&= I+\frac{\sin\theta}{\theta}[\boldsymbol{\phi}]_\times
+\frac{1-\cos\theta}{\theta^2}[\boldsymbol{\phi}]_\times^2, \\[0.4em]
\mathrm{Log}(R)
&= \frac{\theta}{2\sin\theta}\,(R-R^\top)^\vee,
\quad
\theta=\arccos\!\big((\mathrm{tr}\,R-1)/2\big).
\end{aligned}
$$

- **$\boldsymbol{\phi}$**：切空间坐标；名义更新 $R\leftarrow R\,\mathrm{Exp}(\delta\boldsymbol{\theta})$。

**反对称映射.** $[\mathbf{a}]_\times$（`SKEW_SYM_MATRX`）：

$$
[\mathbf{a}]_\times
=
\begin{bmatrix}
0 & -a_z & a_y \\
a_z & 0 & -a_x \\
-a_y & a_x & 0
\end{bmatrix},
\qquad
[\mathbf{a}]_\times\mathbf{b}=\mathbf{a}\times\mathbf{b}.
$$

**有时程积分.** 代码 `Exp(ω, dt)`：

$$
R\leftarrow R\,\mathrm{Exp}(\boldsymbol{\omega}\,dt).
$$

**SE(3) 与右扰动.** PoseGraph 中 `Se3Log` 存 $(\mathbf{t},\,\mathrm{Log}\,R)$：

$$
T
=
\begin{bmatrix}
R & \mathbf{t} \\
\mathbf{0}^\top & 1
\end{bmatrix},
\qquad
T\oplus\boldsymbol{\xi}
=
T\,\mathrm{Exp}_{\mathrm{SE}(3)}(\boldsymbol{\xi}),
\quad
\boldsymbol{\xi}=(\boldsymbol{\rho},\boldsymbol{\phi})\in\mathbb{R}^6.
$$

---

## 2. 状态定义（19 维）

**名义状态.** `StatesGroup`；$\tau=$ `inv_expo_time`（曝光倒数）：

$$
\mathbf{x}
=
\big\{
R_{\mathrm{end}},\,
\mathbf{p}_{\mathrm{end}},\,
\tau,\,
\mathbf{v}_{\mathrm{end}},\,
\mathbf{b}_g,\,
\mathbf{b}_a,\,
\mathbf{g}
\big\}.
$$

**误差状态布局.** $\delta\mathbf{x}\in\mathbb{R}^{19}$，下标与代码一致；协方差 $P\in\mathbb{R}^{19\times19}$。

| 下标 | 符号 | 维 | 含义 |
|:----:|:----:|:--:|------|
| $0{:}2$ | $\delta\boldsymbol{\theta}$ | 3 | 姿态误差 |
| $3{:}5$ | $\delta\mathbf{p}$ | 3 | 位置 |
| $6$ | $\delta\tau$ | 1 | 曝光倒数 |
| $7{:}9$ | $\delta\mathbf{v}$ | 3 | 速度 |
| $10{:}12$ | $\delta\mathbf{b}_g$ | 3 | 陀螺偏置 |
| $13{:}15$ | $\delta\mathbf{b}_a$ | 3 | 加计偏置 |
| $16{:}18$ | $\delta\mathbf{g}$ | 3 | 重力 |

**盒加**（`operator+=`）：

$$
\begin{aligned}
R &\leftarrow R\,\mathrm{Exp}(\delta\boldsymbol{\theta}), \\
\mathbf{p} &\leftarrow \mathbf{p}+\delta\mathbf{p}, \\
\tau &\leftarrow \tau+\delta\tau, \\
\mathbf{v} &\leftarrow \mathbf{v}+\delta\mathbf{v}, \\
\mathbf{b}_g &\leftarrow \mathbf{b}_g+\delta\mathbf{b}_g, \\
\mathbf{b}_a &\leftarrow \mathbf{b}_a+\delta\mathbf{b}_a, \\
\mathbf{g} &\leftarrow \mathbf{g}+\delta\mathbf{g}.
\end{aligned}
$$

**盒减**（`operator-`）：

$$
\delta\mathbf{x}
=
\mathbf{x}_a\boxminus\mathbf{x}_b
=
\begin{bmatrix}
\mathrm{Log}(R_b^\top R_a) \\
\mathbf{p}_a-\mathbf{p}_b \\
\tau_a-\tau_b \\
\mathbf{v}_a-\mathbf{v}_b \\
\mathbf{b}_{g,a}-\mathbf{b}_{g,b} \\
\mathbf{b}_{a,a}-\mathbf{b}_{a,b} \\
\mathbf{g}_a-\mathbf{g}_b
\end{bmatrix}.
$$

- **$R_{\mathrm{end}},\mathbf{p}_{\mathrm{end}}$**：扫描 / 图像对齐时刻的 IMU 位姿。
- **$\tau$**：光度曝光系数倒数；纯 LIO 更新通常不改正该项。

---

## 3. IMU 传播

### 3.1 传感器模型

世界系重力 $\mathbf{g}$，IMU 系测量：

$$
\begin{aligned}
\boldsymbol{\omega}_m &= \boldsymbol{\omega}+\mathbf{b}_g+\mathbf{n}_g, \\
\mathbf{a}_m &= R^\top(\mathbf{a}-\mathbf{g})+\mathbf{b}_a+\mathbf{n}_a.
\end{aligned}
$$

偏置随机游走 $\dot{\mathbf{b}}_g=\mathbf{n}_{bg}$，$\dot{\mathbf{b}}_a=\mathbf{n}_{ba}$。实现中加速度先按重力模长归一化再减偏置。

### 3.2 中值积分

相邻样本 $(k,k+1)$（`UndistortPcl`）：

$$
\begin{aligned}
\boldsymbol{\omega}
&=
\tfrac12\big(\boldsymbol{\omega}_m^{(k)}+\boldsymbol{\omega}_m^{(k+1)}\big)-\mathbf{b}_g, \\[0.3em]
\mathbf{a}
&=
\tfrac12\big(\mathbf{a}_m^{(k)}+\mathbf{a}_m^{(k+1)}\big)
\cdot\frac{g_0}{\lVert\bar{\mathbf{a}}\rVert}
-\mathbf{b}_a.
\end{aligned}
$$

离散更新（步长 $\Delta t$）：

$$
\begin{aligned}
R &\leftarrow R\,\mathrm{Exp}(\boldsymbol{\omega}\,\Delta t), \\
\mathbf{v} &\leftarrow \mathbf{v}+\big(R\mathbf{a}+\mathbf{g}\big)\Delta t, \\
\mathbf{p} &\leftarrow \mathbf{p}+\mathbf{v}\,\Delta t
+\tfrac12\big(R\mathbf{a}+\mathbf{g}\big)\Delta t^2.
\end{aligned}
$$

代码对位置常用前向欧拉 $\mathbf{p}\leftarrow\mathbf{p}+\mathbf{v}\Delta t$（与 FAST-LIVO2 一致）。

### 3.3 误差状态转移

线性化 $F_x=\partial\mathbf{x}_{k+1}/\partial\mathbf{x}_k$，主要非零块：

$$
\begin{aligned}
F_{RR}&=\mathrm{Exp}(-\boldsymbol{\omega}\Delta t), &
F_{R,b_g}&=-I\,\Delta t, \\
F_{pv}&=I\,\Delta t, &
F_{vR}&=-R[\mathbf{a}]_\times\Delta t, \\
F_{v,b_a}&=-R\,\Delta t, &
F_{vg}&=I\,\Delta t.
\end{aligned}
$$

协方差：

$$
P\leftarrow F_x\,P\,F_x^\top+Q.
$$

- **$Q$**：`cov_w`，对角块 $\propto\Delta t^2$（陀螺 / 加计 / 偏置 RW / 可选 $\tau$）。
- **加计噪声（世界系）**：$R\,\mathrm{diag}(\sigma_a^2)\,R^\top\Delta t^2$。
- **LO**：`ForwardWithoutImu` 用简化 $F_x$ 与可选运动补偿。

### 3.4 去畸变与外参

扫描内点相对帧末偏移 $\Delta t_j$：

$$
p^{\mathrm{undist}}
=
\mathrm{Exp}(\boldsymbol{\omega},-\Delta t_j)\,p
-R^\top\mathbf{v}\,\Delta t_j.
$$

得 `feats_undistort_`，再下采样为 `feats_down_body_`。LiDAR → IMU：

$$
\mathbf{p}_I=R_{IL}\,p_L+\mathbf{t}_{IL}
\quad(\texttt{extR\_},\,\texttt{extT\_}).
$$

---

## 4. 点–面 ESIKF 更新

### 4.1 残差

名义位姿 $(R,\mathbf{t})=(R_{\mathrm{end}},\mathbf{p}_{\mathrm{end}})$。平面：单位法向 $\mathbf{n}$、中心 $\mathbf{q}$、不确定 $\Sigma_\pi$。

$$
\mathbf{p}^w
=
R\,\mathbf{p}_I+\mathbf{t}
=
R\big(R_{IL}p_L+\mathbf{t}_{IL}\big)+\mathbf{t}.
$$

有符号点–面距离；量测取 $z=-r$：

$$
r=\mathbf{n}^\top(\mathbf{p}^w-\mathbf{q}),
\qquad
z=-r.
$$

- **$r$**：`dis_to_plane_`；$z$ 使正增量对应减小距离。

### 4.2 点与平面噪声

体噪声 $\Sigma_b=\mathrm{calcBodyCov}(p;\sigma_{\mathrm{dept}},\sigma_{\mathrm{beam}})$。残差构建用世界系：

$$
\Sigma_p
=
R\,\Sigma_b\,R^\top
+\big(-[\mathbf{p}_I]_\times\big)\,P_{RR}\,\big(-[\mathbf{p}_I]_\times\big)^\top
+P_{pp}.
$$

量测方差分支（代码）：

$$
\Sigma_p^{(\mathrm{meas})}
=
R\,R_{IL}\,\Sigma_b\,(R\,R_{IL})^\top.
$$

平面参数贡献与标量信息权重：

$$
\begin{aligned}
\mathbf{J}_{nq}
&=
\big[(\mathbf{p}^w-\mathbf{q})^\top,\;-\mathbf{n}^\top\big], \\[0.3em]
\sigma_\ell
&=
\mathbf{J}_{nq}\,\Sigma_\pi\,\mathbf{J}_{nq}^\top, \\[0.3em]
R_i^{-1}
&=
\big(
0.001+\sigma_\ell+\mathbf{n}^\top\Sigma_p^{(\mathrm{meas})}\mathbf{n}
\big)^{-1}.
\end{aligned}
$$

### 4.3 雅可比

对 $(\delta\boldsymbol{\theta},\delta\mathbf{p})$。令 $\mathbf{A}=[\mathbf{p}_I]_\times R^\top\mathbf{n}$：

$$
\mathbf{H}_i
=
\big[\mathbf{A}^\top,\;\mathbf{n}^\top\big]
\in\mathbb{R}^{1\times6}.
$$

- **$\mathbf{A}$**：姿态扰动灵敏度（`point_crossmat * R^T * n`）。
- **$\mathbf{n}$**：位置扰动灵敏度。

有效特征数 $m$。堆叠 $H\in\mathbb{R}^{m\times6}$，$\mathbf{z}\in\mathbb{R}^m$，$W=\mathrm{diag}(R_1^{-1},\ldots,R_m^{-1})$：

$$
\begin{aligned}
\mathcal{I}_6
&=
H^\top W H
=
\sum_{i=1}^m R_i^{-1}\,\mathbf{H}_i^\top\mathbf{H}_i
\in\mathbb{R}^{6\times6}, \\[0.3em]
\mathbf{b}_6
&=
H^\top W\mathbf{z}
=
\sum_{i=1}^m R_i^{-1}\,\mathbf{H}_i^\top z_i.
\end{aligned}
$$

嵌入 $19\times19$：仅左上 $6\times6$ 非零（`H_T_H` / `HTz`）。

### 4.4 迭代更新

传播态 $\mathbf{x}^{\mathrm{prop}}$，线性化点 $\mathbf{x}$：

$$
\mathcal{I}_{19}
=
\begin{bmatrix}
\mathcal{I}_6 & 0 \\
0 & 0
\end{bmatrix},
\qquad
K_1
=
\big(\mathcal{I}_{19}+P^{-1}\big)^{-1}.
$$

$$
G_{:,1:6}
=
K_1
\begin{bmatrix}
I_6 \\
0
\end{bmatrix}
\mathcal{I}_6.
$$

相对误差 $\mathbf{v}=\mathbf{x}^{\mathrm{prop}}\boxminus\mathbf{x}$，增量：

$$
\delta\mathbf{x}
=
K_1
\begin{bmatrix}
\mathbf{b}_6 \\
\mathbf{0}
\end{bmatrix}
+\mathbf{v}
-G
\begin{bmatrix}
\mathbf{v}_{0:5} \\
\mathbf{0}
\end{bmatrix}.
$$

更新 $\mathbf{x}\leftarrow\mathbf{x}\boxplus\delta\mathbf{x}$。收敛（度 / cm）：

$$
\lVert\delta\boldsymbol{\theta}\rVert\cdot\frac{180}{\pi}<0.01^\circ
\quad\land\quad
\lVert\delta\mathbf{p}\rVert\cdot 100<1.5\,\mathrm{cm}.
$$

收敛或达 `max_iterations` 后：

$$
P\leftarrow(I-G)\,P.
$$

- **含义**：仅前 $6$ 维直接进量测；速度 / 偏置 / 重力经 $P$ 交叉项间接修正。
- **重匹配**：每轮 `BuildResidualListOMP`；`rematch_num` 保证至少两次有效匹配后再锁 $P$。

---

## 5. 退化、门控与强度

### 5.1 位置 FIM

第 $0$ 次迭代：

$$
\mathrm{FIM}_{\mathrm{pos}}
=
(\mathcal{I}_6)_{3:5,\,3:5}
=
\sum_{i=1}^m R_i^{-1}\,\mathbf{n}_i\mathbf{n}_i^\top
\in\mathbb{R}^{3\times3}.
$$

特征分解 $\mathrm{FIM}_{\mathrm{pos}}=U\,\mathrm{diag}(\lambda_1,\lambda_2,\lambda_3)\,U^\top$：

$$
\kappa
=
\frac{\lambda_{\max}}{\lambda_{\min}}
\quad
\big(\lambda_{\min}\le\varepsilon\Rightarrow\kappa=\infty\big).
$$

弱方向 $\mathbf{u}$ 为最小特征向量。判定：

$$
\texttt{is_degenerated}
=
\big(\lambda_{\min}<\tau_\lambda\big)
\;\lor\;
\big(\kappa>\tau_\kappa\big).
$$

配置键：`eigenvalue_threshold`、`condition_number_threshold`。

### 5.2 各向异性膨胀

$$
P_{pp}\leftarrow P_{pp}+\alpha\,\mathbf{u}\mathbf{u}^\top,
\qquad
P_{pp}=P_{3:5,\,3:5}.
$$

- **$\alpha$**：`inflation_scale`；强度辅助时 $\alpha\leftarrow\alpha\cdot\gamma$（`inflation_multiplier`）。

### 5.3 ConfidenceGating

若 `lidar_valid` 为假：跳过本帧 LiDAR 更新（保留 IMU 传播）。否则：

$$
\mathcal{I}_6\leftarrow w_L\,\mathcal{I}_6,
\qquad
\mathbf{b}_6\leftarrow w_L\,\mathbf{b}_6,
\qquad
w_L\in[w_{\min},\,1].
$$

动态比例 $\rho_{\mathrm{dyn}}$、健康超时、退化强度均可压低 $w_L$ 或置无效。

---

## 6. 视觉直接法（VIO）

### 6.1 坐标系

外参 $(R_{CI},\mathbf{t}_{CI})$（`Rcl` / `Pcl`）。IMU 位姿 $(R_{WI},\mathbf{p}_{WI})$：

$$
\begin{aligned}
R_{CW} &= R_{CI}\,R_{WI}^\top, \\
\mathbf{t}_{CW} &= -R_{CI}\,R_{WI}^\top\mathbf{p}_{WI}+\mathbf{t}_{CI}.
\end{aligned}
$$

路标 $\mathbf{X}$（世界）→ 相机：

$$
\mathbf{p}_c=R_{CW}\mathbf{X}+\mathbf{t}_{CW}.
$$

### 6.2 投影

针孔，归一化平面 $(x,y)=(X/Z,\,Y/Z)$：

$$
\begin{bmatrix} u \\ v \end{bmatrix}
=
\begin{bmatrix} f_x & 0 \\ 0 & f_y \end{bmatrix}
\begin{bmatrix} x \\ y \end{bmatrix}
+
\begin{bmatrix} c_x \\ c_y \end{bmatrix}.
$$

等距 / 全向由 `AbstractCamera::world2cam` 实现；$J_\pi=\partial(u,v)/\partial\mathbf{p}_c$。

### 6.3 光度残差

曝光 $\tau=$ `inv_expo_time`；patch 边长 $s$，点数 $N_{\mathrm{pt}}$，维数 $H_{\mathrm{DIM}}=N_{\mathrm{pt}}\,s^2$：

$$
r_{uv}
=
\tau_{\mathrm{ref}}\,I_{\mathrm{ref}}(u,v)
-\tau\,I_{\mathrm{cur}}(u',v').
$$

$$
\min\sum_{(u,v)\in\mathcal{P}} r_{uv}^2.
$$

金字塔尺度 $2^\ell$，由粗到细（`UpdateState` / `UpdateStateInverse`）。

### 6.4 雅可比与更新

图像梯度 $J_{\mathrm{img}}=\nabla I_{\mathrm{cur}}$：

$$
\frac{\partial r}{\partial\boldsymbol{\xi}}
=
-\tau\,J_{\mathrm{img}}\,J_\pi\,
\frac{\partial\mathbf{p}_c}{\partial\boldsymbol{\xi}}.
$$

$\boldsymbol{\xi}$ 含 $(\delta\boldsymbol{\theta},\delta\mathbf{p})$，逆构图可含 $\delta\tau$。信息形式：

$$
K
=
\big(
H^\top H+(P/\sigma_I^2)^{-1}
\big)^{-1},
\qquad
\sigma_I^2=\texttt{img\_point\_cov}.
$$

外点：残差 $>\,\texttt{outlier\_threshold}$；动态 mask 在特征选取阶段生效。

---

## 7. Scan Context

**描述子.** 环数 $N_r$、扇区 $N_s$，格内高度统计：

$$
S\in\mathbb{R}^{N_r\times N_s}.
$$

绕竖直轴旋转 ↔ 列循环移位 $\mathrm{Shift}(S,\Delta)$。

**检索.**

$$
d(S_q,S_j)
=
\min_{\Delta}\,
\mathrm{dist}\big(S_q,\,\mathrm{Shift}(S_j,\Delta)\big).
$$

Top-$K$ 后经 `MultiModalConsensus` 得置信度 $c$；若 $c\ge c_{\min}$ 则加 PoseGraph 回环边。

---

## 8. PoseGraph（SE(3) Gauss–Newton）

**相对误差.** 节点 $T_i$，测量 $\tilde{T}_{ij}$，信息 $\omega$：

$$
\mathbf{e}_{ij}
=
\mathrm{Log}\big(
\tilde{T}_{ij}^{-1}\,T_i^{-1}T_j
\big)
\in\mathbb{R}^6.
$$

$$
\min_{\{T_k\}}
\sum_{(i,j)}
\omega_{ij}\,\lVert\mathbf{e}_{ij}\rVert_2^2.
$$

**正规方程.** 右扰动有限差分雅可比 $J_i,J_j\in\mathbb{R}^{6\times6}$：

$$
H\delta=-\mathbf{b},
$$

$$
H
\leftarrow
\sum\omega
\begin{bmatrix}
J_i^\top J_i & J_i^\top J_j \\
J_j^\top J_i & J_j^\top J_j
\end{bmatrix},
\qquad
\mathbf{b}
\leftarrow
\sum\omega
\begin{bmatrix}
J_i^\top\mathbf{e} \\
J_j^\top\mathbf{e}
\end{bmatrix}.
$$

锚定 gauge 后 $T\leftarrow T\oplus\delta$（无 Ceres）。

**注入 ESIKF.**

$$
T\leftarrow\Delta T\,T,
\qquad
P\leftarrow P+\varepsilon I.
$$

跨会话：大 `node_id_base`；链式因子刷新 $T_{\mathrm{world}\leftarrow\mathrm{session}}$。

---

## 9. BnB / 3D-BBS

先验占据 $M:\mathbb{R}^3\to\{0,1\}$。候选 $T$，查询点 $\{p_k\}$：

$$
s(T)
=
\sum_k
\mathbf{1}\big[M(T\,p_k)=1\big].
$$

粗层上界 $\bar{s}<s^*$ 则剪枝；细层精化。

| `search_space` | 参数化 |
|:--------------:|--------|
| `se2` | $(x,y,\psi)$ |
| `se3_yaw` | $(x,y,z,\psi)$ |
| `se3` | 上者 + 局部 $(\phi,\theta)$ |

接受：$s(T)\ge\texttt{score\_threshold}$；预算 `max_depth` / `max_expansions`。注入时替换 $(R,\mathbf{p})$ 并重置位置协方差。

---

## 10. 地图置信度

高质量观测：$c\leftarrow c+\Delta_+$，$N\leftarrow N+1$。退化下：$c\leftarrow\beta\,c-\Delta_-$。剪枝：

$$
\big(c<\tau_c\big)
\land
\big(N<N_{\min}\big)
\;\Rightarrow\;
\text{删除根体素}.
$$

$N\ge N_{\min}$ 的锚点即使 $c$ 暂时偏低也保留。

---

## 11. 轨迹评测（Umeyama，无尺度）

TUM：$(t,\,t_x,t_y,t_z,\,q_x,q_y,q_z,q_w)$。时间关联 $|\hat{t}-t^{\mathrm{gt}}|\le\Delta t_{\max}$。

**中心化.**

$$
\begin{aligned}
\bar{\mathbf{p}}&=\tfrac1N\sum\hat{\mathbf{p}}_i, &
\bar{\mathbf{q}}&=\tfrac1N\sum\mathbf{p}_i^{\mathrm{gt}}, \\
X_i&=\hat{\mathbf{p}}_i-\bar{\mathbf{p}}, &
Y_i&=\mathbf{p}_i^{\mathrm{gt}}-\bar{\mathbf{q}}.
\end{aligned}
$$

**正交 Procrustes.**

$$
W
=
\tfrac1N\sum_i Y_i X_i^\top
=
U\Sigma V^\top,
\qquad
R=UV^\top
$$

（若 $\det R<0$，翻转 $U$ 最后一列）。平移与 ATE：

$$
\mathbf{t}=\bar{\mathbf{q}}-R\bar{\mathbf{p}},
$$

$$
\mathbf{e}_i=R\hat{\mathbf{p}}_i+\mathbf{t}-\mathbf{p}_i^{\mathrm{gt}},
\qquad
\mathrm{ATE}_{\mathrm{RMSE}}
=
\sqrt{\tfrac1N\sum_i\lVert\mathbf{e}_i\rVert_2^2}.
$$

实现：`plot_traj_ate.umeyama_se3`、`eval_pose_traj`。

---

## 12. 符号速查

| 符号 | 含义 |
|:----:|------|
| $R,\mathbf{p},\mathbf{v}$ | IMU 姿态 / 位置 / 速度 |
| $\mathbf{b}_g,\mathbf{b}_a,\mathbf{g}$ | 陀螺偏置 / 加计偏置 / 重力 |
| $\tau$ | 曝光倒数 `inv_expo_time` |
| $P$ | $19\times19$ 协方差 |
| $\mathbf{n},\mathbf{q},r$ | 平面法向 / 中心 / 点面距离 |
| $\mathcal{I}_6,H,W$ | $6$ 维信息、雅可比、量测权重 |
| $w_L,\mathbf{u},\alpha$ | LiDAR 门控权重、弱方向、膨胀系数 |
| $S,d$ | Scan Context 矩阵与距离 |
| $T_i,\mathbf{e}_{ij}$ | 位姿图节点与边误差 |
| $s(T)$ | BnB 占据命中分 |
| $\mathrm{ATE}_{\mathrm{RMSE}}$ | 绝对轨迹误差 |
