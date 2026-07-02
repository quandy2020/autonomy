(prediction-math)=
# 3. 数学原理

> 本文以 **Step-by-Step** 方式推导 Prediction 模块涉及的核心公式。运动模型实现见 [06_motion_models.md](06_motion_models.md)；算法综述见 [09_survey.md](09_survey.md)。

---

### 3.1 问题形式化

#### Step 1：定义障碍状态

时刻 $t$ 第 $i$ 个动态障碍的状态：

$$
\mathbf{x}_i(t) = \big(x, y, v_x, v_y, \theta, \omega, \ldots\big)^\top \in \mathbb{R}^n
$$

#### Step 2：预测目标

给定历史观测 $\mathcal{H}_t = \{\mathbf{z}_{i,t-k}, \ldots, \mathbf{z}_{i,t}\}$ 与预测时域 $T_p$，求未来轨迹：

$$
\hat{\mathbf{x}}_i(t+\tau) = f\big(\mathbf{x}_i(t), \tau, \mathcal{H}_t\big), \quad \tau \in [0, T_p]
$$

#### Step 3：输出形式

离散预测轨迹：

$$
\hat{\mathcal{T}}_i = \big\{ \hat{\mathbf{x}}_i(t + k \Delta t) \big\}_{k=0}^{K}, \quad K = T_p / \Delta t
$$

---

### 3.2 恒速模型（Constant Velocity, CV）

#### Step 1：状态向量

$$
\mathbf{x} = [x, y, v_x, v_y]^\top
$$

#### Step 2：连续时间动力学

$$
\dot{x} = v_x, \quad \dot{y} = v_y, \quad \dot{v}_x = 0, \quad \dot{v}_y = 0
$$

#### Step 3：状态转移矩阵

$$
F = \begin{bmatrix}
1 & 0 & \Delta t & 0 \\
0 & 1 & 0 & \Delta t \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

#### Step 4：离散预测

$$
\mathbf{x}_{k+1} = F \mathbf{x}_k
$$

展开：

$$
\begin{aligned}
x_{k+1} &= x_k + v_{x,k} \Delta t \\
y_{k+1} &= y_k + v_{y,k} \Delta t \\
v_{x,k+1} &= v_{x,k} \\
v_{y,k+1} &= v_{y,k}
\end{aligned}
$$

---

### 3.3 恒转弯率模型（CTRV）

#### Step 1：状态向量

$$
\mathbf{x} = [x, y, v, \theta, \omega]^\top
$$

其中 $v$ 为线速度，$\theta$ 为航向，$\omega$ 为角速度。

#### Step 2：非线性动力学

$$
\begin{aligned}
\dot{x} &= v \cos\theta \\
\dot{y} &= v \sin\theta \\
\dot{v} &= 0 \\
\dot{\theta} &= \omega \\
\dot{\omega} &= 0
\end{aligned}
$$

#### Step 3：解析积分（$\omega \neq 0$）

$$
\begin{aligned}
x(t+\Delta t) &= x + \frac{v}{\omega}\big(\sin(\theta + \omega\Delta t) - \sin\theta\big) \\
y(t+\Delta t) &= y + \frac{v}{\omega}\big(-\cos(\theta + \omega\Delta t) + \cos\theta\big) \\
\theta(t+\Delta t) &= \theta + \omega \Delta t
\end{aligned}
$$

#### Step 4：$\omega \to 0$ 极限（退化为 CV）

$$
x(t+\Delta t) = x + v\cos\theta \cdot \Delta t, \quad
y(t+\Delta t) = y + v\sin\theta \cdot \Delta t
$$

---

### 3.4 恒加速度模型（CA）

#### Step 1：状态向量

$$
\mathbf{x} = [x, y, v_x, v_y, a_x, a_y]^\top
$$

#### Step 2：状态转移矩阵

$$
F = \begin{bmatrix}
1 & 0 & \Delta t & 0 & \frac{\Delta t^2}{2} & 0 \\
0 & 1 & 0 & \Delta t & 0 & \frac{\Delta t^2}{2} \\
0 & 0 & 1 & 0 & \Delta t & 0 \\
0 & 0 & 0 & 1 & 0 & \Delta t \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$

#### Step 3：预测公式

$$
\begin{aligned}
x_{k+1} &= x_k + v_{x,k}\Delta t + \frac{1}{2}a_{x,k}\Delta t^2 \\
v_{x,k+1} &= v_{x,k} + a_{x,k}\Delta t
\end{aligned}
$$

---

### 3.5 卡尔曼滤波预测步

#### Step 1：预测（Prior）

$$
\begin{aligned}
\hat{\mathbf{x}}_{k|k-1} &= F \hat{\mathbf{x}}_{k-1|k-1} \\
P_{k|k-1} &= F P_{k-1|k-1} F^\top + Q
\end{aligned}
$$

#### Step 2：更新（Posterior）

$$
\begin{aligned}
K_k &= P_{k|k-1} H^\top (H P_{k|k-1} H^\top + R)^{-1} \\
\hat{\mathbf{x}}_{k|k} &= \hat{\mathbf{x}}_{k|k-1} + K_k (\mathbf{z}_k - H \hat{\mathbf{x}}_{k|k-1}) \\
P_{k|k} &= (I - K_k H) P_{k|k-1}
\end{aligned}
$$

#### Step 3：多步前向预测

对 $j = 1, \ldots, K$：

$$
\hat{\mathbf{x}}_{k+j|k} = F^j \hat{\mathbf{x}}_{k|k}, \quad
P_{k+j|k} = F^j P_{k|k} (F^j)^\top + \sum_{i=0}^{j-1} F^i Q (F^i)^\top
$$

---

### 3.6 数据关联（匈牙利算法）

#### Step 1：代价矩阵

$N$ 个轨迹、$M$ 个检测，代价：

$$
C_{ij} = 1 - \mathrm{IoU}(\mathrm{track}_i, \mathrm{det}_j)
$$

或采用欧氏距离代价：

$$
C_{ij} = \|\hat{\mathbf{x}}_i - \mathbf{z}_j\|_2
$$

#### Step 2：最优分配

求排列 $\pi$ 使 $\sum_i C_{i,\pi(i)}$ 最小，满足一对一匹配。

#### Step 3：门控（Gating）

马氏距离门控，剔除不可能的关联：

$$
d_M^2 = (\mathbf{z} - H\hat{\mathbf{x}})^\top S^{-1} (\mathbf{z} - H\hat{\mathbf{x}}) < \chi^2_{\alpha, df}
$$

其中 $S = H P H^\top + R$。

---

### 3.7 轨迹不确定性表示

#### 3.7.1 高斯椭圆

2D 位置协方差 $P_{xy}$ 的特征值 $\lambda_1, \lambda_2$ 定义 $k\sigma$ 置信椭圆：

$$
(\mathbf{p} - \boldsymbol{\mu})^\top P_{xy}^{-1} (\mathbf{p} - \boldsymbol{\mu}) = k^2
$$

#### 3.7.2 多模态轨迹

对 $L$ 条假设轨迹 $\{\hat{\mathcal{T}}^{(l)}, \pi_l\}$，$\sum_l \pi_l = 1$：

$$
P(\mathcal{T}^{(l)}) = \pi_l, \quad \hat{\mathbf{x}}^{(l)}(t+\tau) = f_l(\mathbf{x}, \tau)
$$

---

### 3.8 碰撞风险评估

#### Step 1：自车预测轨迹 $\hat{\mathcal{T}}_{ego}$

#### Step 2：障碍预测轨迹 $\hat{\mathcal{T}}_i$

#### Step 3：时空重叠

$$
\mathrm{risk} = \max_{k} \mathbf{1}_{\left\{\|\hat{\mathbf{p}}_{ego,k} - \hat{\mathbf{p}}_{i,k}\| < r_{\mathrm{safe}}\right\}}
$$

或基于 TTC（Time To Collision）：

$$
\mathrm{TTC} = \min_{\tau > 0} \big\{ \tau : \|\mathbf{p}_{ego}(\tau) - \mathbf{p}_i(\tau)\| < r_{safe} \big\}
$$

---

### 3.9 符号表

| 符号 | 含义 |
|------|------|
| $T_p$ | 预测时域 |
| $\Delta t$ | 预测/滤波时间步 |
| $F$ | 状态转移矩阵 |
| $H$ | 观测矩阵 |
| $Q, R$ | 过程/观测噪声协方差 |
| $\omega$ | 角速度（CTRV） |
| $\pi_l$ | 多模态轨迹概率 |
