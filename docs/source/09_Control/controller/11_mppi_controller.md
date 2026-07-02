(mppi-controller)=
# 11. MPPI Controller

> 归属 [§8 局部控制器 · §8.3](../08_controller_algorithms.md#83-mppi-controller)
>
> **Model Predictive Path Integral**（MPPI）是 Nav2 `nav2_mppi_controller` 的随机 MPC 局部控制器：对 $K$ 条扰动控制序列前向仿真，用 Critics 打分后经 **softmax 加权**输出最优速度。  
> 公共推导见 [03_math.md §3.8](../03_math.md)；与 DWB 对比见 [09_survey.md §9.5.5](../09_survey.md)；配置见 `config/control/controller.lua`。

| 维度 | 说明 |
|------|------|
| 类型 | 随机 MPC / 路径积分控制 |
| 输入 | 全局路径、位姿、速度、costmap |
| 输出 | $(v_x^{cmd}, v_y^{cmd}, \omega_z^{cmd})$ |
| 预测时域 | $H \times \Delta t$（默认 $56 \times 0.05 = 2.8$ s） |
| Autonomy 状态 | ⏳ 配置预留，插件未实现 |

---

## 1. 文献与演进

### 1.1 核心论文清单

以下五篇（类）文献构成理解 MPPI 的**最小阅读集**。按理论依赖排列：先读路径积分随机最优控制（PI²），再读信息论 MPC 推导，然后读 MPPI 实时控制实践，最后用 Nav2 集成与工程评测落地。

---

#### ① 《Learning Policy Improvements with Path Integrals》(2010)

| 字段 | 内容 |
|------|------|
| **作者** | E. Theodorou, J. Buchli, S. Schaal |
| **出处** | *ICML*, 2010；扩展版 *JMLR* 11:3137–3181 |
| **核心价值** | **PI² 算法鼻祖**。将策略改进转化为路径积分近似，用 rollout 采样 + 指数加权更新控制，无需梯度、无需矩阵求逆。 |
| **对应本文** | [§3 Step 2](#step-2-softmax) 指数加权 · [§3 Step 3](#step-3-monte-carlo) 采样扰动 |

**PI² 核心思想**（与 MPPI 一脉相承）：

| 步骤 | PI² | MPPI（本文） |
|------|-----|--------------|
| 1 | 在名义控制上加高斯噪声 rollout | Step 3 采样 $u_t^{(k)} = \bar{u}_t + \epsilon$ |
| 2 | 用轨迹代价指数加权更新控制 | Step 5 softmax 权重 $w_k$ |
| 3 | 迭代直至收敛 | 每周期 1 次（`iteration_count=1`）+ warm-start |

---

#### ② 《Information Theoretic MPC for Model-Based Reinforcement Learning》(2017)

| 字段 | 内容 |
|------|------|
| **作者** | G. Williams, N. Wagener, B. Goldfain, P. Drew, J. M. Rehg, E. A. Theodorou, B. Boots |
| **出处** | *IEEE ICRA*, 2017 |
| **核心价值** | 用**信息论**（KL 散度、自由能）推导采样 MPC 控制律，摆脱控制仿射假设；支持神经网络动力学。 |
| **对应本文** | [§3 Step 1](#step-1) 代价泛函 · [§3 Step 2](#step-2-softmax) 控制律推导 |

**关键结论**：最优控制可写为轨迹代价的指数加权期望：

$$
u^* = \frac{\mathbb{E}_{\epsilon}\big[u \cdot \exp(-S(u)/\lambda)\big]}{\mathbb{E}_{\epsilon}\big[\exp(-S(u)/\lambda)\big]}
$$

$\lambda$ 即 Nav2 参数 `temperature`。

---

#### ③ 《Model Predictive Path Integral Control》(2018)

| 字段 | 内容 |
|------|------|
| **作者** | G. Williams, A. Aldrich, B. Goldfain, E. A. Theodorou |
| **出处** | *IEEE IROS*, 2018 |
| **核心价值** | 正式提出 **MPPI** 名称与 receding-horizon 框架：batch 并行采样、GPU 加速、非凸非光滑代价均可。 |
| **对应本文** | [§3 Step 4–7](#step-4) 仿真 · [§6 伪代码](#6-完整算法伪代码) |

**与 PI² 的区别**：

| 维度 | PI² | MPPI |
|------|-----|------|
| 时域 | 开环策略改进 | 有限时域 MPC + 滚动执行 $u_0$ |
| 热启动 | 迭代多轮 | 上周期解左移 + 末位补噪声 |
| 实时性 | 离线/慢迭代 | 单周期内 $K$ 条并行 rollout |

---

#### ④ Nav2 `nav2_mppi_controller` 工程实现

| 字段 | 内容 |
|------|------|
| **维护** | ROS Navigation2 社区（`ros-navigation/navigation2`） |
| **出处** | [nav2_mppi_controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller) · [官方配置文档](https://docs.nav2.org/configuration/packages/configuring-mppic.html) |
| **核心价值** | 将 MPPI 落地为 **ControllerServer 插件**：Critic 插件链、DiffDrive/Omni/Ackermann 运动模型、xsimd 向量化 batch 仿真。 |
| **对应本文** | [§2 架构](#2-架构) · [§5 Critics](#5-critics-详解) · [§7 配置](#7-配置参考) |

**Nav2 实现要点**（摘自官方文档）：

-  modest CPU（4th gen i5）上可达 **100+ Hz**；推荐 `batch_size=2000` @ 30 Hz 或 `1000` @ 50 Hz
- `iteration_count` 建议保持 **1**，优先增大 `batch_size` 而非迭代轮数
- Critics 不必凸、不必可微——与 DWB 一样可插拔组合

---

#### ⑤ 《The Marathon 2: A Navigation System》及工程评测 (2020+)

| 字段 | 内容 |
|------|------|
| **作者** | S. Macenski, et al. (2020, IROS) |
| **核心价值** | Nav2 系统架构：MPPI 作为可选局部控制器，与 BT、costmap 协同。 |
| **工程评测** | Urrea (2024/2026) 农业场景 **DWB vs MPPI vs RPP**；Elbouhy (2025) ROS2 局部规划横向对比 |
| **对应本文** | [§8 调参](#8-调参要点) · [§9 与 DWB 对比](#9-与-dwb-对比) |

**评测典型结论**（多篇归纳）：

| 维度 | MPPI | DWB |
|------|------|-----|
| 动态避障 | ✅ 预测时域长，绕障更前瞻 | ✅ 成熟，短 horizon 反应快 |
| 路径跟踪 | 需调 PathAlign 权重 | PathDist/Align Critic 直观 |
| 计算量 | $O(K \cdot H)$，5–20 ms | $O(N_v N_\omega K)$，1–5 ms |
| 全向 / Ackermann | 原生支持 `vy`、转弯半径约束 | 扩展困难 |
| 参数敏感度 | `temperature`、`gamma`、噪声 $\Sigma$ | Critic 权重 |

---

### 1.2 演进脉络

```
2005  Kappen           路径积分随机最优控制理论
        │
2010  Theodorou et al.  PI²（采样 + 指数加权策略改进）
        │
2017  Williams et al.   信息论 MPC → 采样控制律闭式表达
        │
2018  Williams et al.   MPPI 命名 + receding-horizon + GPU
        │
2022+ nav2_mppi_controller  Nav2 插件化 Critics + 运动模型
        │
2024+ 农业/工业评测      MPPI vs DWB/TEB/RPP 场景化选型
```

| 阶段 | 来源 | 贡献 |
|------|------|------|
| **路径积分 SOC** | Kappen, 2005；Theodorou, 2010 | 随机 HJB → 路径积分，PI² |
| **信息论 MPC** | Williams et al., 2017 | KL / 自由能推导，$\lambda$ 温度参数 |
| **MPPI** | Williams et al., 2018 | 滚动时域 MPC + batch 采样 |
| **Nav2 集成** | `nav2_mppi_controller` | Critic 插件、三种运动模型 |
| **工程评测** | Urrea, Elbouhy 等 | 与 DWB/RPP 横向基准 |

### 1.3 推荐阅读路径

| 目标 | 阅读顺序 |
|------|----------|
| 理解 softmax 公式 | ② Williams 2017 → 本文 §3 Step 1–2 |
| 理解 MPC 滚动执行 | ③ Williams 2018 → 本文 §3 Step 6–7 |
| 理解 Nav2 Critics | ④ 官方文档 → 本文 §5 |
| 选型与调参 | ⑤ 评测论文 → 本文 §8–§9 |

---

## 2. 架构

MPPI 由四层组成：**噪声采样 → 批量前向仿真 → Critic 评分 → softmax 加权输出**。

```
FollowPath
    │
    ▼
┌──────────────────────────────────────────────────────────┐
│  MPPIController (nav2_mppi_controller)                    │
│  ┌──────────────┐   ┌─────────────────────────────────┐ │
│  │ NoiseGenerator│→│ 控制序列扰动 {U⁽¹⁾…U⁽ᴷ⁾}         │ │
│  │  N(0, Σ)      │   └──────────────┬──────────────────┘ │
│  └──────────────┘                  │                     │
│                                    ▼                     │
│  ┌─────────────────────────────────────────────────────┐ │
│  │ MotionModel (DiffDrive / Omni / Ackermann)          │ │
│  │ 批量 rollout H 步 → 轨迹张量 X⁽ᵏ⁾                   │ │
│  └───────────────────────┬─────────────────────────────┘ │
│                          ▼                               │
│  ┌─────────────────────────────────────────────────────┐ │
│  │ CriticManager（插件链，可组合权重）                  │ │
│  │ Constraint · Cost · PathAlign · Goal · …            │ │
│  └───────────────────────┬─────────────────────────────┘ │
│                          ▼                               │
│              wₖ = softmax(-S⁽ᵏ⁾/λ)  →  u* = Σ wₖ u⁽ᵏ⁾ │
└──────────────────────────┬───────────────────────────────┘
                           ▼
                    cmd_vel (vx, vy, ωz)
```

### 2.1 与 Autonomy 的映射

| Nav2 组件 | Autonomy 对应 | 说明 |
|-----------|---------------|------|
| `nav2_mppi_controller::MPPIController` | `ControllerInterface` 子类 | 待实现 |
| `mppi::Optimizer` | 控制器内部模块 | 采样 + softmax + warm-start |
| `mppi::MotionModel` | 运动学插件 | DiffDrive / Omni / Ackermann |
| `mppi::critics::CriticFunction` | Critic 接口 | 可复用 Nav2 设计 |
| `Costmap2D` | `Costmap2DWrapper` | 已有 |
| `nav2_core::GoalChecker` | `GoalChecker` | 已有 |

### 2.2 单周期数据流

```
Step A  读取 (pose, velocity, path, costmap)
Step B  warm-start：上周期控制序列左移，末位补噪声
Step C  对 K 条序列加高斯扰动，批量仿真 H 步
Step D  每条轨迹经所有 Critic 累加代价 S⁽ᵏ⁾
Step E  softmax 加权得 u*，执行 u₀*
Step F  （可选）iteration_count > 1 时重复 B–E
```

---

## 3. 数学原理（Step-by-Step）

> 与 [03_math.md §3.8](../03_math.md) 公共部分一致；本文补充推导动机与 Nav2 实现细节。

(step-1)=
### Step 1：随机最优控制问题

在预测时域 $H$ 上求控制序列 $U = \{u_0, \ldots, u_{H-1}\}$，最小化：

$$
J(U) = \phi(x_H) + \sum_{t=0}^{H-1} \left( q(x_t, u_t) + \frac{\gamma}{2}\, u_t^\top \Sigma^{-1} u_t \right)
$$

| 符号 | 含义 | Nav2 参数 |
|------|------|-----------|
| $x_t$ | 状态 $(x, y, \theta)$ | 仿真输出 |
| $u_t$ | 控制 $(v_x, v_y, \omega_z)$ | 采样变量 |
| $q(x_t, u_t)$ | 运行代价（Critics 之和） | 各 Critic `cost_weight` |
| $\phi(x_H)$ | 终端代价 | 通常含在 GoalCritic 中 |
| $\gamma$ | 控制正则强度 | `gamma = 0.015` |
| $\Sigma$ | 采样协方差 | `vx_std`, `vy_std`, `wz_std` |

约束 $x_{t+1} = f(x_t, u_t)$，$f$ 由运动模型插件定义。

**正则项动机**：$\frac{\gamma}{2} u^\top \Sigma^{-1} u$ 惩罚偏离名义控制 $\bar{u}$ 过大的扰动，等价于高斯先验 $\bar{u} \sim \mathcal{N}(0, \Sigma/\gamma)$，防止采样爆炸。

(step-2-softmax)=
### Step 2：从路径积分到 Softmax

Williams (2017) 用信息论推导：在约束 $\mathrm{KL}(q \| p) \leq \epsilon$ 下最小化自由能，得到最优控制为轨迹分布下的指数加权期望。

定义第 $k$ 条采样轨迹总代价 $S^{(k)} = S(U^{(k)})$，Monte Carlo 近似：

$$
u_t^* \approx \sum_{k=1}^{K} w_k\, u_t^{(k)}, \quad
w_k = \frac{\exp\!\left(-\dfrac{1}{\lambda}(S^{(k)} - S_{\min})\right)}{\sum_{j=1}^{K} \exp\!\left(-\dfrac{1}{\lambda}(S^{(j)} - S_{\min})\right)}
$$

| 参数 | 效果 |
|------|------|
| $\lambda \to 0$ | 权重集中于最低代价轨迹（近似 argmin） |
| $\lambda \to \infty$ | 权重趋于均匀（过度探索） |
| 减 $S_{\min}$ | 数值稳定，不改变 $w_k$ 比例 |

Nav2 默认 `temperature = 0.3`。

(step-3-monte-carlo)=
### Step 3：Monte Carlo 采样与正则项

以 warm-start 序列 $\bar{U}$ 为中心，加独立高斯扰动：

$$
u_t^{(k)} = \mathrm{clip}\big(\bar{u}_t + \epsilon_t^{(k)},\; \mathcal{U}_{limits}\big), \quad
\epsilon_t^{(k)} \sim \mathcal{N}(0, \Sigma)
$$

$$
\Sigma = \mathrm{diag}(\sigma_{v_x}^2,\; \sigma_{v_y}^2,\; \sigma_{\omega_z}^2)
$$

Autonomy 默认：$\sigma_{v_x} = \sigma_{v_y} = 0.2$，$\sigma_{\omega_z} = 0.4$，$K = 2000$。

**单条轨迹代价**（含正则）：

$$
S^{(k)} = \sum_{t=0}^{H-1} \left( \sum_{c} w_c\, \mathcal{L}_c(x_t^{(k)}, u_t^{(k)}) + \frac{\gamma}{2}\, {u_t^{(k)}}^\top \Sigma^{-1} u_t^{(k)} \right)
$$

(step-4)=
### Step 4：前向仿真与运动模型

对每条序列 $t = 0, \ldots, H-1$ 递推 $x_{t+1}^{(k)} = f(x_t^{(k)}, u_t^{(k)})$。

**DiffDrive**（Autonomy 默认 `motion_model = "DiffDrive"`）：

$$
\begin{aligned}
x_{t+1} &= x_t + v_x \cos\theta_t \cdot \Delta t \\
y_{t+1} &= y_t + v_x \sin\theta_t \cdot \Delta t \\
\theta_{t+1} &= \theta_t + \omega_z \cdot \Delta t
\end{aligned}
$$

**Omni**（`vy_std > 0` 时启用横向速度）：

$$
\begin{aligned}
x_{t+1} &= x_t + (v_x \cos\theta_t - v_y \sin\theta_t)\,\Delta t \\
y_{t+1} &= y_t + (v_x \sin\theta_t + v_y \cos\theta_t)\,\Delta t \\
\theta_{t+1} &= \theta_t + \omega_z \cdot \Delta t
\end{aligned}
$$

**Ackermann**（`AckermannConstraints.min_turning_r`）：

$$
|v_x| \leq \omega_z \cdot R_{min} \quad \text{（转弯半径约束）}
$$

### Step 5：Critics 累加

各 Critic 独立计算 $\mathcal{L}_c$，加权求和。最终代价：

$$
S^{(k)} \mathrel{+}= w_c \cdot \mathcal{L}_c^{\;p_c}
$$

$p_c$ = `cost_power`（默认 1，即线性）。

### Step 6：输出与 warm-start

执行首步控制：

$$
v_x^{cmd} = u_0^{*,x}, \quad v_y^{cmd} = u_0^{*,y}, \quad \omega_z^{cmd} = u_0^{*,z}
$$

**warm-start**（下一周期）：

$$
\bar{u}_t \leftarrow u_{t+1}^*, \quad t = 0, \ldots, H-2; \qquad
\bar{u}_{H-1} \leftarrow \bar{u}_{H-1} + \epsilon
$$

要求 `controller_frequency = 1 / model_dt`（20 Hz ↔ 0.05 s）。

### Step 7：数值示例

| 步骤 | 值 |
|------|-----|
| 3 条轨迹代价 | $S^{(1)}=10,\; S^{(2)}=8,\; S^{(3)}=12$ |
| $S_{\min}=8,\; \lambda=0.3$ | — |
| 未归一化权重 | $\tilde{w}_1=e^{-6.67},\; \tilde{w}_2=1,\; \tilde{w}_3=e^{-13.33}$ |
| 归一化后 | $w_2 \approx 0.88$（最优轨迹主导） |

---

## 4. 运动模型

| 模型 | 插件 | 控制维度 | Autonomy 配置 |
|------|------|----------|---------------|
| DiffDrive | `mppi::DiffDriveMotionModel` | $(v_x, \omega_z)$，`vy=0` | `motion_model = "DiffDrive"` |
| Omni | `mppi::OmniMotionModel` | $(v_x, v_y, \omega_z)$ | `vy_std > 0` |
| Ackermann | `mppi::AckermannMotionModel` | $(v_x, \omega_z)$ + $R_{min}$ | `AckermannConstraints.min_turning_r` |

**速度硬约束**（采样后 clip）：

$$
v_x \in [v_{x,\min}, v_{x,\max}], \quad
v_y \in [-v_{y,\max}, v_{y,\max}], \quad
\omega_z \in [-\omega_{z,\max}, \omega_{z,\max}]
$$

Autonomy 默认：$v_{x,\max}=0.5$，$v_{x,\min}=-0.35$，$\omega_{z,\max}=1.9$ rad/s。

---

## 5. Critics 详解

Nav2 MPPI Critics 与 DWB 语义相近，但作用于**整条预测轨迹**而非单条短 rollout。下表对齐 Autonomy `controller.lua` 默认配置。

| Critic | 代价含义 | 默认权重 | 激活条件 |
|--------|----------|----------|----------|
| **ConstraintCritic** | 速度/加速度越界惩罚 | 4.0 | 始终 |
| **CostCritic** | 沿轨迹 costmap 代价累积 | 3.81 | 始终 |
| **PathAlignCritic** | 轨迹与路径**弧长对齐**后的横向偏差 | 14.0 | 距路径起点 > 0.5 m |
| **PathFollowCritic** | 轨迹终点沿路径前进程度 | 5.0 | 距目标 > 1.4 m |
| **PathAngleCritic** | 轨迹朝向与路径切向偏差 | 2.0 | 距路径 > 0.5 m |
| **GoalCritic** | 轨迹点距目标位置 | 5.0 | 距目标 < 1.4 m |
| **GoalAngleCritic** | 轨迹朝向与目标航向差 | 3.0 | 距目标 < 0.5 m |
| **PreferForwardCritic** | 惩罚 $v_x < 0$ | 5.0 | 距目标 < 0.5 m |
| ObstaclesCritic | 距障碍斥力（可选） | — | 默认 **关闭** |
| TwirlingCritic | 惩罚过大 $\|\omega_z\|$ | — | 默认关闭 |

### 5.1 典型 Critic 公式

**CostCritic**（沿轨迹每 `trajectory_point_step` 采样）：

$$
\mathcal{L}_{cost} = \sum_{t \in \mathcal{T}_{sample}} c(x_t, y_t)
$$

若任一点 $c \geq c_{critical}$（默认 300），整条轨迹判无效（`collision_cost = 1e6`）。

**PathAlignCritic**（Nav2 2023+ 弧长对齐，非简单最近点）：

1. 预计算路径各点弧长 $s_i^{path}$
2. 对轨迹点 $p_t$，按其已行进弧长 $s_t^{traj}$ 在路径上找对应点 $p_{match}$
3. 累加偏差：

$$
\mathcal{L}_{align} = \sum_{t} \big\| p_t - p_{match}(s_t^{traj}) \big\|_2
$$

可选 `use_path_orientations` 叠加航向差。`max_path_occupancy_ratio` 过大时自动降权，让 CostCritic 接管绕障。

**GoalCritic**（接近目标时激活）：

$$
\mathcal{L}_{goal} = w \cdot \| p_t - p_{goal} \|_2, \quad \text{当 } \|p_{robot} - p_{goal}\| < d_{thresh}
$$

**PreferForward**：

$$
\mathcal{L}_{fwd} = w \cdot \max(0,\; -v_x)
$$

**ConstraintCritic**（越界量二次惩罚）：

$$
\mathcal{L}_{constraint} = \sum_{t} \Big( [\max(0, v - v_{max})]^2 + [\max(0, v_{min} - v)]^2 + \cdots \Big)
$$

### 5.2 Critics 协作时序

```
远离目标                    接近目标
│                           │
├─ PathFollow（沿路径前进）  ├─ GoalCritic（位置）
├─ PathAlign（横向对齐）     ├─ GoalAngleCritic（航向）
├─ PathAngle（朝向）         ├─ PreferForward（禁止后退）
├─ CostCritic（避障）        └─ CostCritic（仍生效）
└─ ConstraintCritic（始终）
```

`threshold_to_consider` 控制各 Critic 的**空间激活半径**，应与预测时域 $H \cdot \Delta t$ 协调，保证 PathFollow → Goal 平滑交接。

---

## 6. 完整算法伪代码

```
function MPPICompute(pose, vel, path, costmap, U_bar):
    // Step B: warm-start
    U_bar ← ShiftLeft(U_bar)                              // 左移一位
    U_bar[H-1] ← U_bar[H-1] + Noise(Σ)                    // 末位补噪声

    trajectories ← []
    costs ← []

    for k = 1 to K:                                       // Step C
        U_k ← Clip(U_bar + GaussianNoise(Σ), vel_limits)
        X_k ← Rollout(pose, U_k, H, dt, motion_model)     // Step 4
        trajectories.append(X_k)

    for k = 1 to K:
        S_k ← 0
        for critic in critics:
            S_k += critic.weight * critic.score(X_k, U_k, path, costmap)  // Step 5
        for t = 0 to H-1:
            S_k += (gamma/2) * U_k[t]^T * Σ^{-1} * U_k[t]   // 正则项
        costs.append(S_k)

    S_min ← min(costs)
    for k = 1 to K:                                       // Step 2
        w_k ← exp(-(costs[k] - S_min) / lambda)

    U_star ← Σ_k w_k * U_k / Σ_k w_k                      // softmax 加权
    U_bar ← U_star                                        // 保存 warm-start

    return U_star[0]                                        // Step 6: 执行 u_0
```

**复杂度**：$O(K \cdot H \cdot C)$，$C$ 为 Critic 数。默认 $2000 \times 56 \approx 1.1 \times 10^5$ 次状态推进/周期，桌面 CPU 约 5–20 ms；可用 xsimd / GPU 加速。

---

## 7. 配置参考

Autonomy `config/control/controller.lua` 中 `mppi_controller` 段（与 Nav2 官方默认对齐）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `time_steps` | 56 | 预测步数 $H$ |
| `model_dt` | 0.05 s | 仿真步长 $\Delta t$ |
| `batch_size` | 2000 | 采样数 $K$ |
| `iteration_count` | 1 | 每周期 MPPI 迭代次数 |
| `temperature` | 0.3 | softmax 温度 $\lambda$ |
| `gamma` | 0.015 | 控制正则 $\gamma$ |
| `vx_std` / `vy_std` / `wz_std` | 0.2 / 0.2 / 0.4 | 噪声标准差 |
| `vx_max` / `vx_min` | 0.5 / -0.35 m/s | 线速度范围 |
| `wz_max` | 1.9 rad/s | 角速度上限 |
| `motion_model` | `"DiffDrive"` | 运动模型 |
| `controller_frequency` | 20 Hz | **必须** $= 1/\text{model\_dt}$ |

**Critics 列表**（`controller.lua`）：

```lua
critics = {
    "ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic",
    "PathAlignCritic", "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic",
}
```

**关键 Critic 参数摘录**：

```lua
PathAlignCritic = {
    cost_weight = 14.0,
    max_path_occupancy_ratio = 0.05,
    trajectory_point_step = 4,
    threshold_to_consider = 0.5,
    offset_from_furthest = 20,
},
CostCritic = {
    cost_weight = 3.81,
    critical_cost = 300.0,
    collision_cost = 1000000.0,
    trajectory_point_step = 2,
},
GoalCritic = {
    cost_weight = 5.0,
    threshold_to_consider = 1.4,  -- 建议 ≈ 预测时域距离
},
```

---

## 8. 调参要点

| 现象 | 可能原因 | 调整 |
|------|----------|------|
| 贴障 / 碰撞 | Cost 权重低或未启用 footprint | 增大 `CostCritic.cost_weight`；`consider_footprint = true` |
| 不跟路径、切弯 | PathAlign 权重低 | 增大 `PathAlignCritic`（默认 14.0 已较高） |
| 接近目标摆动 | Goal / GoalAngle 权重过高 | 略降 `GoalCritic`；检查 `threshold_to_consider` 交接 |
| 原地抖动 / 空转 | 探索过大 | 启用 `TwirlingCritic`；增大 `gamma` |
| 反应迟钝 | 采样不足或 $\lambda$ 过小 | 增大 `batch_size`；略增 `temperature` |
| 轨迹发散 | $\lambda$ 过大或 $\Sigma$ 过大 | 减小 `temperature` 或 `vx_std` / `wz_std` |
| 计算超时 | $K \cdot H$ 过大 | 减 `batch_size`（→1000）或 `time_steps`（→40） |
| 后退过多 | 未惩罚负 $v_x$ | 启用 `PreferForwardCritic` |

**频率对齐检查清单**：

$$
f_{ctrl} = \frac{1}{\Delta t} \quad \Leftrightarrow \quad
\text{controller\_frequency} = 20 \;\land\; \text{model\_dt} = 0.05
$$

---

## 9. 与 DWB 对比

| 维度 | MPPI | DWB |
|------|------|-----|
| 搜索空间 | 控制**序列** $(u_0, \ldots, u_{H-1})$ | 单步速度 $(v, \omega)$ |
| 选优方式 | softmax 加权平均 | argmin 单条最优 |
| 预测时域 | 显式 $H$ 步（2.8 s） | `sim_time`（~1.7 s） |
| 热启动 | 序列左移 | 无（每周期独立采样） |
| 非凸代价 | 天然支持 | 支持（Critic 插件） |
| 计算量 | 较高（batch 并行） | 较低 |
| 全向机器人 | Omni 模型原生 | 需扩展采样维度 |

两者 Critics 语义可对照迁移，详见 [§13 DWB Controller §4](13_dwb_controller.md#4-critics)。

---

## 10. Autonomy 移植清单

```
□ 继承 ControllerInterface
□ 实现 Optimizer（采样 + softmax + warm-start）
□ 实现 MotionModel 插件（先 DiffDrive，再 Omni/Ackermann）
□ 实现/移植 Critic 插件接口（对齐 Nav2 8 个默认 Critic）
□ 接入 Costmap2DWrapper（CostCritic / footprint）
□ xsimd 或 OpenMP 并行 batch rollout
□ controller.lua 已就绪，接 AUTOLINK_PLUGIN 注册
□ 单元测试：直线跟踪、动态绕障、窄通道、目标旋转、warm-start 连续性
```

---

## 11. 参考文献

### 11.1 核心论文（必读）

| # | 文献 | 链接 |
|---|------|------|
| ① | Theodorou, E., Buchli, J., & Schaal, S. (2010). *Learning Policy Improvements with Path Integrals*. ICML / JMLR 11. | [JMLR](https://jmlr.org/papers/v11/theodorou10a.html) |
| ② | Williams, G., et al. (2017). *Information Theoretic MPC for Model-Based Reinforcement Learning*. ICRA. | [PDF](https://homes.cs.washington.edu/~bboots/files/InformationTheoreticMPC.pdf) |
| ③ | Williams, G., Aldrich, A., Goldfain, B., & Theodorou, E. A. (2018). *Model Predictive Path Integral Control*. IROS. | [Georgia Tech ACDS](https://sites.gatech.edu/acds/mppi/) |
| ④ | Macenski, S., et al. (2020). *The Marathon 2: A Navigation System*. IROS. | [DOI](https://doi.org/10.1109/IROS45743.2020.9341207) |

### 11.2 工程评测与对比

| 文献 | 对比对象 | 场景 |
|------|----------|------|
| Urrea (2024). *Autonomous Navigation in Agriculture Scenarios using Nav2* | DWB vs MPPI vs RPP | 农业 Gazebo |
| Urrea et al. (2026). *Systems*, 14(3) | DWB vs RPP vs MPPI | 半结构化环境 |
| Elbouhy et al. (2025). *Comparative Analysis of Local Trajectory Planning Algorithms in ROS2*. SIMPAR. | DWB vs MPPI vs RPP | 动静态障碍 |

### 11.3 官方文档与源码

| 资源 | 链接 |
|------|------|
| Nav2 MPPI 配置 | [configuring-mppic](https://docs.nav2.org/configuration/packages/configuring-mppic.html) |
| `nav2_mppi_controller` | [GitHub](https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller) |
| PathAlign 优化 PR #3872 | [ros-navigation/navigation2#3872](https://github.com/ros-navigation/navigation2/pull/3872) |

### 11.4 BibTeX

```bibtex
@inproceedings{theodorou2010pi2,
  author    = {Theodorou, Evangelos and Buchli, Jonas and Schaal, Stefan},
  title     = {Learning Policy Improvements with Path Integrals},
  booktitle = {Proceedings of the 27th International Conference on Machine Learning},
  year      = {2010}
}

@inproceedings{williams2017information,
  author    = {Williams, Grady and Wagener, Nikolas and Goldfain, Brian and Drews, Paul and Rehg, James M. and Theodorou, Evangelos A. and Boots, Byron},
  title     = {Information Theoretic {MPC} for Model-Based Reinforcement Learning},
  booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
  year      = {2017}
}

@inproceedings{williams2018mppi,
  author    = {Williams, Grady and Aldrich, Andrew and Goldfain, Brian and Theodorou, Evangelos A.},
  title     = {Model Predictive Path Integral Control},
  booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year      = {2018}
}

@inproceedings{macenski2020marathon2,
  author    = {Macenski, Steven and Mart{\'\i}n, Francisco and White, Ruffin and Gin{\'e}s Clavero, Jonatan},
  title     = {The Marathon 2: A Navigation System},
  booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year      = {2020}
}
```
