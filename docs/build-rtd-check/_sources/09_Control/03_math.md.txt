(control-math)=
# 3. 数学原理

> 本文以 **Step-by-Step** 方式推导 Control 模块涉及的全部核心公式。算法对比见 [09_survey.md](09_survey.md)；Checker 实现细节见 [06_checkers.md](06_checkers.md)；VelocitySmoother 代码对照见 [07_velocity_smoother.md](07_velocity_smoother.md)。

---

### 3.1 问题形式化

#### Step 1：定义状态与控制

机器人在平面环境中，时刻 $t$ 的状态：

$$
x(t) = \big(x(t),\; y(t),\; \theta(t),\; v_x(t),\; v_y(t),\; \omega_z(t)\big)^\top
$$

控制输入（局部控制器输出）：

$$
u(t) = \big(v_x^{cmd},\; v_y^{cmd},\; \omega_z^{cmd}\big)^\top
$$

全局路径 $\mathcal{P} = \{p_0, p_1, \ldots, p_N\}$，$p_i = (x_i, y_i, \theta_i) \in SE(2)$，由 `planning` 模块提供。

#### Step 2：写出最优控制目标

局部控制可建模为有限时域最优控制：

$$
\min_{u(\cdot)} \; J = \int_0^T \Big( e_p^\top Q e_p + w_\theta \, e_\theta^2 + u^\top R u \Big) \, dt
$$

| 符号 | 含义 |
|------|------|
| $e_p = [x - x_{ref},\; y - y_{ref}]^\top$ | 位置误差 |
| $e_\theta = \mathrm{AngleDiff}(\theta, \theta_{ref})$ | 航向误差 |
| $Q \succeq 0$ | 位置误差权重矩阵 |
| $R \succ 0$ | 控制能量权重矩阵 |
| $T$ | 控制时域（FollowPath 直到到达 $p_N$） |

#### Step 3：列出硬约束

$$
\begin{aligned}
&(x, y) \in \mathcal{C}_{\mathrm{free}} \\
&|v_x| \leq v_{x,\max},\; |v_y| \leq v_{y,\max},\; |\omega_z| \leq \omega_{\max} \\
&|a| \leq a_{\max},\; |\dot\omega| \leq \dot\omega_{\max} \\
&p(T) \approx p_N
\end{aligned}
$$

分别对应：不碰撞、速度界、加速度界（由 `VelocitySmoother` 执行）以及终端条件。

#### Step 4：AngleDiff 的定义

Autonomy 使用 `autonomy::common::math::AngleDiff(a, b)`，保证结果 $\in (-\pi, \pi]$：

$$
\mathrm{AngleDiff}(a, b) = \mathrm{Normalize}\big(\mathrm{wrap}(a - b)\big)
$$

其中 $\mathrm{wrap}(\phi) = \phi - 2\pi \lfloor (\phi + \pi) / 2\pi \rfloor$。Goal Checker 中即 $e_\theta = \mathrm{AngleDiff}(\theta_q, \theta_g)$。

---

### 3.2 运动学模型

#### 3.2.1 差速驱动（DiffDrive）

**Step 1：本体坐标系速度**

差速机器人在本体坐标系下 $v_y^{body} = 0$（非完整约束）。本体速度 $(v_x^{body}, 0, \omega_z)$ 与世界坐标系关系：

$$
\begin{bmatrix} v_x^{world} \\ v_y^{world} \end{bmatrix}
=
\begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}
\begin{bmatrix} v_x^{body} \\ 0 \end{bmatrix}
=
\begin{bmatrix} v_x^{body} \cos\theta \\ v_x^{body} \sin\theta \end{bmatrix}
$$

**Step 2：连续时间状态方程**

$$
\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}
=
\begin{bmatrix} \cos\theta \\ \sin\theta \\ 0 \end{bmatrix} v_x
+
\begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} \omega_z
$$

**Step 3：前向欧拉离散化**（离散步长 $\Delta t$ 对应参数 `model_dt`）

$$
\begin{aligned}
x_{k+1} &= x_k + v_x \cos\theta_k \cdot \Delta t \\
y_{k+1} &= y_k + v_x \sin\theta_k \cdot \Delta t \\
\theta_{k+1} &= \theta_k + \omega_z \cdot \Delta t
\end{aligned}
$$

**Step 4：验证非完整约束**

对离散轨迹求 $\Delta y \cos\theta - \Delta x \sin\theta$，在非完整系统下恒为零——轨迹无法侧向滑动，这是 Pure Pursuit / Stanley 等控制器必须"先转向再前进"的根本原因。

#### 3.2.2 全向驱动（Holonomic）

**Step 1：旋转矩阵**

$$
R(\theta) = \begin{bmatrix}
\cos\theta & -\sin\theta & 0 \\
\sin\theta &  \cos\theta & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

**Step 2：状态方程**

$$
\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot{\theta} \end{bmatrix}
= R(\theta) \begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix}
$$

**Step 3：与 DiffDrive 对比**

| 特性 | DiffDrive | Holonomic |
|------|-----------|-----------|
| 自由度 | 2 ($v_x, \omega_z$) | 3 ($v_x, v_y, \omega_z$) |
| 可侧移 | ❌ | ✅ |
| MPPI `motion_model` | `"DiffDrive"` | 需设 `vy_std > 0` |

#### 3.2.3 Ackermann 约束

**Step 1：几何关系**

Ackermann 车辆转弯时，前后轮延长线交于瞬时旋转中心 ICR。轴距 $L$，前轮转角 $\delta$：

$$
R = \frac{L}{\tan\delta}, \quad \omega_z = \frac{v_x}{R} = \frac{v_x \tan\delta}{L}
$$

**Step 2：最小转弯半径约束**

$$
|\omega_z| \leq \frac{|v_x|}{R_{\min}}
\quad \Leftrightarrow \quad
|\delta| \leq \arctan\!\frac{L}{R_{\min}}
$$

Autonomy 配置 `AckermannConstraints.min_turning_r = 0.2` m 即 $R_{\min}$。

---

### 3.3 Lookahead 点选取（`GetLookAheadPoint`）

#### Step 1：路径变换到机器人坐标系

将全局路径 $\mathcal{P}$ 经 TF 变换，使机器人位于原点 $(0,0)$，朝向沿 $x$ 轴。后续所有几何计算在此坐标系下进行。

#### Step 2：沿路径累积弧长

对路径点 $p_0, p_1, \ldots, p_N$，计算相邻点欧氏距离：

$$
\Delta s_i = \|p_i - p_{i-1}\|_2 = \sqrt{(x_i - x_{i-1})^2 + (y_i - y_{i-1})^2}
$$

累积距离 $S_k = \sum_{i=1}^{k} \Delta s_i$。

#### Step 3：找到第一个超出 lookahead 的段

给定前瞻距离 $L_d$，找最小索引 $k$ 使：

$$
S_{k-1} < L_d \leq S_{k-1} + \Delta s_k
$$

若不存在（路径总长 $< L_d$），取末点或外推（`interpolate_after_goal = true` 时）。

#### Step 4：在线段上线性插值

剩余距离：

$$
\delta = L_d - S_{k-1} \quad (0 < \delta \leq \Delta s_k)
$$

插值比例 $\alpha = \delta / \Delta s_k$，lookahead 点：

$$
p_{la} = p_{k-1} + \alpha (p_k - p_{k-1})
$$

代码对应 `LinearInterpolation(prev, curr, interpolation_dist)`，其中插值距离 $\delta$ 即 `interpolation_dist`。

#### Step 5：计算 lookahead 朝向

$$
\theta_{la} = \operatorname{atan2}\big(p_{la,y} - p_{k-1,y},\; p_{la,x} - p_{k-1,x}\big)
$$

#### 数值示例

| 步骤 | 计算 | 结果 |
|------|------|------|
| 路径段 | $p_0=(0,0),\, p_1=(1,0),\, p_2=(2,0.5)$ | — |
| 段长 | $\Delta s_1=1,\, \Delta s_2=\sqrt{1.25}\approx 1.12$ | — |
| $L_d = 1.3$ | $S_1=1 < 1.3 \leq S_1+\Delta s_2$ | 落在段 $[p_1, p_2]$ |
| 插值 | $\delta=0.3,\, \alpha=0.3/1.12\approx 0.27$ | $p_{la}\approx(1.27,\, 0.13)$ |

---

### 3.4 Pure Pursuit 曲率推导

#### Step 1：建立几何模型

机器人位于原点 $O$，朝向 $x$ 轴。Lookahead 点 $P = (x_l, y_l)$（$y_l > 0$）。Pure Pursuit 假设机器人沿**圆弧**从 $O$ 驶向 $P$，圆弧与 $x$ 轴相切于 $O$。

#### Step 2：确定圆心

圆心 $C$ 在 $y$ 轴上（切线垂直于半径）：$C = (0, R)$，$R$ 为曲率半径。

#### Step 3：利用圆方程

$P$ 在圆上，距圆心 $R$：

$$
x_l^2 + (y_l - R)^2 = R^2
$$

#### Step 4：展开并求解 $R$

$$
x_l^2 + y_l^2 - 2 y_l R + R^2 = R^2
\quad \Rightarrow \quad
x_l^2 + y_l^2 = 2 y_l R
$$

$$
R = \frac{x_l^2 + y_l^2}{2 y_l}
$$

#### Step 5：曲率

$$
\kappa = \frac{1}{R} = \frac{2 y_l}{x_l^2 + y_l^2}
$$

#### Step 6：经典近似

当 lookahead 距离 $L_d = \sqrt{x_l^2 + y_l^2}$ 固定时：

$$
\kappa \approx \frac{2 y_l}{L_d^2}
$$

Autonomy `controller_utils` 中圆-线段交点给出 $(x^*, y^*)$，Pure Pursuit 取 $\kappa = 2y^*/L_d^2$。

#### Step 7：角速度命令

$$
\omega_z = \kappa \cdot v_x
$$

#### Step 8：Regulated Pure Pursuit 速度调节（扩展）

Nav2 RPP 在 Step 7 之后追加：

$$
v_x \leftarrow \min\!\left(v_{desired},\; \frac{\omega_{\max}}{|\kappa|},\; \alpha \cdot d_{obs},\; \beta \cdot d_{goal}\right)
$$

Autonomy 尚未实现 RPP 插件，但工具函数已支持 Step 1–7。

---

### 3.5 圆-线段交点推导（`CircleSegmentIntersection`）

#### Step 1：问题设定

圆心在原点，半径 $r$：$x^2 + y^2 = r^2$。

线段端点 $P_1 = (x_1, y_1)$，$P_2 = (x_2, y_2)$。

#### Step 2：参数化直线

$$
x = x_1 + t \cdot dx, \quad y = y_1 + t \cdot dy, \quad t \in [0, 1]
$$

其中 $dx = x_2 - x_1$，$dy = y_2 - y_1$。

#### Step 3：代入圆方程

$$
(x_1 + t \cdot dx)^2 + (y_1 + t \cdot dy)^2 = r^2
$$

#### Step 4：整理为二次方程

令 $dr^2 = dx^2 + dy^2$，展开：

$$
dr^2 \cdot t^2 + 2(x_1 dx + y_1 dy)\, t + (x_1^2 + y_1^2 - r^2) = 0
$$

#### Step 5：判别式形式（代码变量）

定义 $D = x_1 y_2 - x_2 y_1$（叉积），可证明判别式：

$$
\Delta = r^2 \cdot dr^2 - D^2
$$

要求 $\Delta \geq 0$ 才有实交点。

#### Step 6：解析解

$$
t^* = \frac{D \cdot dy \pm dx \sqrt{\Delta}}{dr^2}, \quad
y^* = \frac{-D \cdot dx \pm dy \sqrt{\Delta}}{dr^2}
$$

#### Step 7：选取线段上的交点

Autonomy 用 $\mathrm{sign}(d_2^2 - d_1^2)$ 选择正确分支，其中 $d_1^2 = x_1^2 + y_1^2$，$d_2^2 = x_2^2 + y_2^2$，保证返回**落在线段上**的交点。

最终代码：

$$
x^* = \frac{D \cdot dy + \mathrm{sign}(d_2^2 - d_1^2) \cdot dx \cdot \sqrt{\Delta}}{dr^2}
$$

$$
y^* = \frac{-D \cdot dx + \mathrm{sign}(d_2^2 - d_1^2) \cdot dy \cdot \sqrt{\Delta}}{dr^2}
$$

---

### 3.6 Stanley 控制器

#### Step 1：Frenet 坐标系

以路径最近点 $p_{ref}$ 为原点，路径切向为 $s$ 轴，法向为 $d$ 轴：

$$
e_y = d_{\mathrm{signed}}(q, \mathcal{P})
$$

其中 $d_{\mathrm{signed}}$ 为机器人到路径的有符号横向距离。

$$
e_\theta = \mathrm{AngleDiff}(\theta_{robot}, \theta_{path})
$$

#### Step 2：Stanley 控制律（Thrun et al., 2006）

$$
\delta = e_\theta + \arctan\!\left(\frac{k_e \cdot e_y}{v_x + \varepsilon}\right)
$$

| 项 | 作用 |
|----|------|
| $e_\theta$ | 消除航向偏差 |
| $\arctan(k_e e_y / (v_x + \varepsilon))$ | 消除横向偏差；$v_x$ 大时增益自动减小 |
| $\varepsilon$ | 防止 $v_x = 0$ 时除零 |

#### Step 3：映射到差速机器人

$$
\omega_z = \frac{v_x \tan\delta}{L} \approx \frac{v_x \cdot \delta}{L}
$$

（小角度近似 $\tan\delta \approx \delta$）

#### Step 4：与 Pure Pursuit 对比

| | Pure Pursuit | Stanley |
|---|-------------|---------|
| 误差定义 | Lookahead 几何 | Frenet 横向 + 航向 |
| 低速表现 | 一般 | 更好（$\varepsilon$ 调节） |
| 典型场景 | 室内 AGV | 高速自动驾驶 |

---

### 3.7 DWB（Dynamic Window Approach）

#### Step 1：速度空间采样

在 $(v, \omega)$ 平面上均匀采样 $N_v \times N_\omega$ 个候选速度。

#### Step 2：动态窗口裁剪

当前速度 $(v_c, \omega_c)$，控制周期 $\Delta t$，加速度限制：

$$
\mathcal{V}_d = \left\{
(v, \omega) :
\begin{aligned}
& v \in [v_c - a_{dec}\Delta t,\; v_c + a_{acc}\Delta t] \cap [v_{min}, v_{max}] \\
& \omega \in [\omega_c - \dot\omega_{dec}\Delta t,\; \omega_c + \dot\omega_{acc}\Delta t]
\end{aligned}
\right\}
$$

只有 $\mathcal{V}_d$ 内的采样点可达。

#### Step 3：前向仿真轨迹

对每个 $(v, \omega) \in \mathcal{V}_d$，用 Step 3.2.1 离散模型仿真 $T_s$ 秒：

$$
x_{k+1} = f(x_k, v, \omega), \quad k = 0, \ldots, K-1
$$

得到轨迹 $\{(x_k, y_k, \theta_k)\}_{k=0}^{K}$。

#### Step 4：逐 critic 评分

$$
G(v, \omega) = \sum_i w_i \cdot \mathcal{S}_i(v, \omega)
$$

| Critic | $\mathcal{S}_i$ 含义 |
|--------|---------------------|
| BaseObstacle | 轨迹上最近障碍距离（越大越好） |
| PathDist | 轨迹终点到路径的距离 |
| GoalDist | 轨迹终点到目标的距离 |
| PathAlign | 轨迹终点航向与路径切向对齐度 |
| PreferForward | 惩罚 $v < 0$ |

#### Step 5：选取最优

$$
(v^*, \omega^*) = \arg\max_{(v,\omega) \in \mathcal{V}_d} G(v, \omega)
$$

#### Step 6：输出

$$
v_x^{cmd} = v^*, \quad \omega_z^{cmd} = \omega^*
$$

---

### 3.8 MPPI（Model Predictive Path Integral）

#### Step 1：随机最优控制问题

求最优控制序列 $U = \{u_0, u_1, \ldots, u_{H-1}\}$ 最小化：

$$
J(U) = \phi(x_H) + \sum_{t=0}^{H-1} \Big( q(x_t, u_t) + \frac{\gamma}{2} u_t^\top \Sigma^{-1} u_t \Big)
$$

约束 $x_{t+1} = f(x_t, u_t)$，$\phi(x_H)$ 为终端代价，$q$ 为运行代价（Critics 之和）。

#### Step 2：信息论路径积分

定义最优代价 $J^*$，路径积分控制律：

$$
u^* = \frac{\int u \cdot \exp\!\left(-\dfrac{1}{\lambda} S(u)\right) du}{\int \exp\!\left(-\dfrac{1}{\lambda} S(u)\right) du}
$$

其中 $S(u)$ 为轨迹总代价，$\lambda > 0$ 为温度参数（默认 `temperature = 0.3`）。

#### Step 3：Monte Carlo 近似

采样 $K$ 条扰动控制序列（对应参数 `batch_size`）：

$$
u_t^{(k)} = \bar{u}_t + \epsilon_t^{(k)}, \quad \epsilon_t^{(k)} \sim \mathcal{N}(0, \Sigma)
$$

$$
\Sigma = \mathrm{diag}(\sigma_{v_x}^2, \sigma_{v_y}^2, \sigma_{\omega_z}^2)
$$

Autonomy 默认：$\sigma_{v_x} = 0.2$，$\sigma_{v_y} = 0.2$，$\sigma_{\omega_z} = 0.4$。

#### Step 4：前向仿真每条轨迹

对第 $k$ 条序列，$t = 0, \ldots, H-1$：

$$
\begin{aligned}
x_{t+1}^{(k)} &= f(x_t^{(k)}, u_t^{(k)}) \\
S^{(k)} &= \sum_{t=0}^{H-1} \Big( \underbrace{\sum_{c} w_c \mathcal{L}_c(x_t^{(k)})}_{\mathrm{Critics}} + \frac{\gamma}{2} u_t^{(k)\top} \Sigma^{-1} u_t^{(k)} \Big)
\end{aligned}
$$

其中状态推进函数 $f(\cdot)$ 采用差速驱动（DiffDrive）离散模型。

#### Step 5：Softmax 权重

$$
w_k = \frac{\exp\!\left(-\dfrac{1}{\lambda}(S^{(k)} - S_{\min})\right)}{\sum_{j=1}^{K} \exp\!\left(-\dfrac{1}{\lambda}(S^{(j)} - S_{\min})\right)}
$$

减 $S_{\min}$ 防止数值溢出，不改变权重比例。

#### Step 6：加权平均得最优控制

$$
u_t^* = \sum_{k=1}^{K} w_k \, u_t^{(k)}
$$

实际只执行 $u_0^*$，下一周期 warm-start：将序列左移一位，末位补噪声。

#### Step 7：Critics 逐项展开

**CostCritic**（沿轨迹采样 costmap）：

$$
\mathcal{L}_{cost} = \sum_{t=0}^{H-1} c(x_t, y_t)
$$

若某采样点满足 $c \geq c_{critical}$，则整条轨迹判为无效。

**GoalCritic**（仅在 $d_{goal} < 1.4$ m 时激活）：

$$
\mathcal{L}_{goal} = w \cdot \|p_t - p_{goal}\|_2
$$

**PathAlignCritic**：

$$
\mathcal{L}_{align} = w \cdot \min_{p \in \mathcal{P}} \|p_t - p\|_2
$$

#### Step 8：频率对齐

$$
f_{ctrl} = \frac{1}{\Delta t} \quad \Leftrightarrow \quad f_{ctrl} = 20 \;\land\; \Delta t = 0.05
$$

对应配置 `controller_frequency = 20`、`model_dt = 0.05`。

#### 数值示例（单步）

| 步骤 | 值 |
|------|-----|
| $K=3$ 条轨迹代价 | $S^{(1)}=10,\, S^{(2)}=8,\, S^{(3)}=12$ |
| $S_{\min}=8,\, \lambda=0.3$ | — |
| 权重 | $w_1=e^{-6.67}/Z,\, w_2=1/Z,\, w_3=e^{-13.33}/Z$ |
| $w_2 \approx 0.88$ | 最优轨迹主导加权平均 |

---

### 3.9 Goal Checker 判定

#### 3.9.1 SimpleGoalChecker — 逐步判定

**Step 1**：若 `check_xy_ == true`，计算：

$$
d_{xy}^2 = (x - x_g)^2 + (y - y_g)^2
$$

**Step 2**：若 $d_{xy}^2 > \varepsilon_{xy}^2$，返回 **false**（未到达）。

**Step 3**：若 XY 通过且 `stateful == true`，设 `check_xy_ = false`（锁定 XY）。

**Step 4**：计算航向误差：

$$
\Delta\theta = \mathrm{AngleDiff}(\theta, \theta_g)
$$

**Step 5**：若 $|\Delta\theta| \leq \varepsilon_\theta$，返回 **true**；否则 **false**。

**Step 6**：`Reset()` 时恢复 `check_xy_ = true`。

```
判定流程图：

check_xy_ == true?
  ├─ YES → d_xy² ≤ ε_xy² ?
  │         ├─ NO  → return false
  │         └─ YES → stateful? → check_xy_ = false
  └─ NO  → (跳过 XY)
           |Δθ| ≤ ε_θ ?
             ├─ YES → return true
             └─ NO  → return false
```

#### 3.9.2 PositionGoalChecker

**Step 1**：若 `stateful && position_reached_`，直接返回 **true**。

**Step 2**：计算 $d_{xy}^2$，若 $\leq \varepsilon_{xy}^2$，设 `position_reached_ = true`，返回 **true**。

**Step 3**：否则返回 **false**。全程不检查航向。

#### 3.9.3 StoppedGoalChecker

**Step 1**：调用 `SimpleGoalChecker::IsGoalReached()`，若 false，返回 **false**。

**Step 2**：检查线速度：

$$
v_{trans} = \sqrt{v_x^2 + v_y^2} \leq v_{trans}^{stop}
$$

**Step 3**：检查角速度：

$$
|\omega_z| \leq \omega_{rot}^{stop}
$$

**Step 4**：Step 2 且 Step 3 均满足，返回 **true**。

---

### 3.10 Progress Checker 判定

#### 3.10.1 SimpleProgressChecker

**Step 1**：将 `current_pose` 转为 Pose2D $(x, y, \theta)$。

**Step 2**：若 `baseline_pose_set_ == false`（首次调用），执行 Step 4。

**Step 3**：计算位移：

$$
d = \operatorname{hypot}(x - x_b, y - y_b)
$$

**Step 4**：若 $d > r$（进度半径 `radius_`，默认 0.5 m）或首次调用：

- `ResetBaselinePose(current)` → 更新 $(x_b, y_b) = (x, y)$
- 返回 **true**（有进度）

**Step 5**：若 $d \leq r$，返回 **false**（无进度 → 可能卡住）。

#### 3.10.2 PoseProgressChecker

**Step 3'**（替代 Step 3）：额外计算转角变化：

$$
\Delta\theta = \left|\mathrm{NormalizeAngleDiff}(\theta - \theta_b)\right|
$$

**Step 4'**：若 $d > r$ **或** $\Delta\theta > \Delta\theta_{req}$（默认 0.5 rad），重置基线，返回 **true**。

---

### 3.11 VelocitySmoother 逐步推导

#### Step 1：输入与符号

| 符号 | 含义 |
|------|------|
| $v_{curr}$ | 当前速度（OPEN_LOOP: 上次输出；CLOSED_LOOP: OdomSmoother） |
| $v_{cmd}$ | 控制器原始命令（经 clamp 到 $[v_{min}, v_{max}]$） |
| $f$ | 平滑频率（`smoothing_frequency`，默认 20 Hz） |
| $a > 0$ | 最大加速度 |
| $d < 0$ | 最大减速度（负值） |

#### Step 2：判定加速/减速模式

$$
\mathrm{Accelerating}
\Leftrightarrow |v_{cmd}| \geq |v_{curr}| \;\land\; v_{curr} \cdot v_{cmd} \geq 0
$$

- **加速模式**：$\Delta v_{\max} = a/f$，$\Delta v_{\min} = -a/f$
- **减速模式**：$\Delta v_{\max} = -d/f$，$\Delta v_{\min} = d/f$

> 注意：$d < 0$，故 $-d > 0$，减速模式下 $\Delta v_{\max} > 0$ 仍表示"允许的速度增量上界"在数值上为正。

#### Step 3：计算原始速度差

$$
\Delta v = v_{cmd} - v_{curr}
$$

#### Step 4：单轴 eta（`findEtaConstraint`）

若 $\Delta v > \Delta v_{\max}$：

$$
\eta = \frac{\Delta v_{\max}}{\Delta v} \quad (0 < \eta < 1)
$$

若 $\Delta v < \Delta v_{\min}$：

$$
\eta = \frac{\Delta v_{\min}}{\Delta v} \quad (0 < \eta < 1)
$$

否则 $\eta = -1$（该轴无需缩放）。

#### Step 5：三轴同步 eta（`scale_velocities = true`）

$$
\eta^* = \arg\min_{\eta_i > 0} |1 - \eta_i|
$$

取约束最紧的一轴，保证三轴等比接近期望速度。

#### Step 6：应用约束（`applyConstraints`）

$$
v_{out} = v_{curr} + \operatorname{clamp}(\eta^* \cdot \Delta v,\; \Delta v_{\min},\; \Delta v_{\max})
$$

#### Step 7：Deadband

$$
v_{final} = \begin{cases} 0 & |v_{out}| < v_{\mathrm{deadband}} \\ v_{out} & \mathrm{otherwise} \end{cases}
$$

#### 完整数值示例

设 $f=20$ Hz，$a=2.5$ m/s²，$v_{curr}=0.2$ m/s，$v_{cmd}=1.0$ m/s（加速模式）：

| Step | 计算 | 结果 |
|------|------|------|
| 2 | 加速模式 | $\Delta v_{\max} = 2.5/20 = 0.125$ |
| 3 | $\Delta v = 1.0 - 0.2$ | $0.8$ |
| 4 | $0.8 > 0.125$ | $\eta = 0.125/0.8 = 0.156$ |
| 6 | $v_{out} = 0.2 + 0.156 \times 0.8$ | $0.325$ m/s |
| 7 | deadband=0 | $v_{final} = 0.325$ |

下一周期 $v_{curr}=0.325$，重复 Step 2–7，约 8 个周期（0.4 s）达到 1.0 m/s。

---

### 3.12 OdomSmoother 滑动平均

#### Step 1：维护双端队列 `odom_history_`

每次收到里程计消息，push_back 到队列尾部。

#### Step 2：移除过期样本

设窗口时长 $T_w$（`odom_duration`，默认 0.1 s），若队首时间戳距当前 $> T_w$：

- 从累积和 `odom_cumulate_` 中减去队首速度
- pop_front

#### Step 3：加入新样本

- push_back 新 odom
- 累积和 += 新速度

#### Step 4：计算平均

$$
\bar{v} = \frac{S_{\mathrm{odom}}}{N}, \quad N = |Q_{\mathrm{odom}}|
$$

其中 $S_{\mathrm{odom}}$、`Q_odom` 分别对应 `odom_cumulate_`、`odom_history_`。

对 $(v_x, v_y, v_z, \omega_x, \omega_y, \omega_z)$ 六个分量分别平均。

---

### 3.13 控制流水线

$$
\boxed{\mathrm{Path}}
\xrightarrow{\mathrm{SetPlan}}
\boxed{\mathrm{Controller}}
\xrightarrow{f_{ctrl}\,\mathrm{Hz}}
\boxed{\mathrm{ComputeVelocityCommands}}
\xrightarrow{\mathrm{GoalChecker}}
\boxed{\mathrm{VelocitySmoother}}
\xrightarrow{}
\boxed{\mathrm{cmd}_{\mathrm{vel}}}
$$

最终输出 `cmd_vel` 到底盘执行器。

并行分支（每个控制周期）：

$$
\boxed{\mathrm{ProgressChecker}}
\xrightarrow{d \leq r \;\land\; \Delta\theta \leq \Delta\theta_{req}}
\boxed{\mathrm{FailedToMakeProgress}}
$$

---

### 3.14 算法一览

| 算法 | 核心公式 | Autonomy 状态 |
|------|----------|---------------|
| Pure Pursuit | $\kappa = 2y_l / L_d^2$ | 工具函数 ✅ |
| Regulated PP | PP + 速度调节 | 配置预留 |
| Stanley | $\delta = e_\theta + \arctan(k_e e_y / v)$ | 未实现 |
| DWB | $\arg\max G(v,\omega)$ in $\mathcal{V}_d$ | 未实现 |
| MPPI | $u^* = \sum w_k u^{(k)}$ | 配置预留 |
| Graceful | PP + 减速区 + 碰撞检测 | 配置预留 |
| VelocitySmoother | $v_{out} = v_{curr} + \operatorname{clamp}(\eta \Delta v)$ | 算法 ✅ |
