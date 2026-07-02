(simulation-math)=
# 3. 数学原理

> 本文推导 Autonomy 仿真涉及的运动学模型与数值积分公式。`nav_test` 实现见 [06_nav_test.md](06_nav_test.md)；车辆限幅见 [08_vehicle_stage.md](08_vehicle_stage.md)。

---

### 3.1 差速驱动运动学

#### Step 1：本体速度

差速机器人在本体坐标系下 $v_y^{body} = 0$。控制输入 $u = (v, \omega)^\top$，其中 $v$ 为线速度，$\omega$ 为角速度。

#### Step 2：世界坐标系速度

$$
\begin{bmatrix} \dot{x} \\ \dot{y} \end{bmatrix}
=
\begin{bmatrix} \cos\theta \\ \sin\theta \end{bmatrix} v
$$

$$
\dot{\theta} = \omega
$$

#### Step 3：连续时间状态方程

$$
\frac{d}{dt} \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}
=
\begin{bmatrix} \cos\theta \\ \sin\theta \\ 0 \end{bmatrix} v
+
\begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} \omega
$$

#### Step 4：前向欧拉离散化（nav_test 使用）

`IntegrateDiffDrive` 实现：

$$
\begin{aligned}
x_{k+1} &= x_k + v \cos\theta_k \cdot \Delta t \\
y_{k+1} &= y_k + v \sin\theta_k \cdot \Delta t \\
\theta_{k+1} &= \mathrm{NormalizeAngle}(\theta_k + \omega \cdot \Delta t)
\end{aligned}
$$

其中 $\Delta t =$ `sim_dt`（默认 0.1 s）。

#### Step 5：四元数姿态

航向 $\theta$ 转四元数（绕 $z$ 轴）：

$$
q_z = \sin(\theta/2), \quad q_w = \cos(\theta/2)
$$

---

### 3.2 轮速逆运动学（差速）

轮距 $L$，左右轮半径 $r$，角速度 $\omega_L, \omega_R$：

$$
v = \frac{r(\omega_R + \omega_L)}{2}, \quad
\omega = \frac{r(\omega_R - \omega_L)}{L}
$$

正运动学（轮速指令）：

$$
\omega_R = \frac{v + \omega L/2}{r}, \quad
\omega_L = \frac{v - \omega L/2}{r}
$$

Gazebo 差速插件内部执行此映射。

---

### 3.3 全向驱动（Holonomic）

$$
\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}
= R(\theta) \begin{bmatrix} v_x \\ v_y \\ \omega \end{bmatrix}
$$

$R(\theta)$ 为 2D 旋转矩阵。`KinematicsControl` 对 $v_x, v_y, \omega$ 分别限幅。

---

### 3.4 Ackermann 运动学

轴距 $L$，前轮转角 $\delta$：

$$
\omega = \frac{v \tan\delta}{L}, \quad
R_{min} = \frac{L}{\tan\delta_{max}}
$$

最小转弯半径约束：$|\omega| \leq |v| / R_{min}$。

---

### 3.5 里程计模型

离散时刻 $k$，带噪声的里程计观测：

$$
\mathbf{z}_k = \begin{bmatrix} x_k \\ y_k \\ \theta_k \end{bmatrix} + \mathbf{n}_k, \quad
\mathbf{n}_k \sim \mathcal{N}(0, \Sigma_{odom})
$$

`nav_test` 为理想仿真（无噪声）；扩展时可加高斯噪声。

---

### 3.6 速度限幅（KinematicsControl）

给定模型上限 $v_{max}, \omega_{max}, a_{max}$：

$$
v^{cmd} = \operatorname{clip}(v^{raw}, -v_{rev}, v_{max})
$$

$$
\omega^{cmd} = \operatorname{clip}(\omega^{raw}, -\omega_{max}, \omega_{max})
$$

加速度限幅（离散）：

$$
v_k = \operatorname{clip}\big(v_{k-1} + a_{max}\Delta t, \; v_{min}, v_{max}\big)
$$

---

### 3.7 仿真时间

**仿真时钟**（`use_sim_time:=true`）：

$$
t_{sim} = t_0 + \sum_k \Delta t_{sim}
$$

与墙钟解耦，由 Gazebo `/clock` 或 Autolink `MODE_SIMULATION` 驱动。

---

### 3.8 符号表

| 符号 | 含义 |
|------|------|
| $v, \omega$ | 线速度、角速度 |
| $\Delta t$ | 仿真步长 `sim_dt` |
| $L$ | 轮距 / 轴距 |
| $\theta$ | 航向角 |
| $R_{min}$ | 最小转弯半径 |
