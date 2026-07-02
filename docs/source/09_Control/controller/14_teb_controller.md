(teb-controller)=
# 14. TEB Controller

> 归属 [§8 局部控制器 · §8.6](../08_controller_algorithms.md#86-tebtimed-elastic-band)
>
> **Timed Elastic Band**（TEB）是 ROS `teb_local_planner` / Nav2 可选局部规划器，将路径建模为**弹性带**（位姿序列 + 时间间隔），通过非线性优化同时优化路径形状与时间。  
> 对比见 [09_survey.md §9.5.5](../09_survey.md)。

| 维度 | 说明 |
|------|------|
| 类型 | 时空优化（非线性规划） |
| 输入 | 全局路径、位姿、速度、障碍（点/多边形） |
| 输出 | 时间参数化轨迹 → $(v_x, \omega_z)$ |
| Autonomy 状态 | ❌ 未实现 |

---

## 1. 核心论文清单

#### ① 《Integrated Online Trajectory Optimization…》(2017)

| 字段 | 内容 |
|------|------|
| **作者** | C. Rösmann, F. Hoffmann, T. Bertram |
| **出处** | *IEEE RAM*, 24(3), 40–48 |
| **核心价值** | **TEB 系统论文**。弹性带 + 时间最优 + 多目标代价，支持动态障碍。 |
| **对应本文** | [§3](#3-数学原理-step-by-step) |

#### ② 《Trajectory Modification Considering Dynamic Constraints…》(2013)

| 字段 | 内容 |
|------|------|
| **作者** | C. Rösmann, et al. |
| **出处** | *IEEE ETFA* |
| **核心价值** | TEB 早期公式：在轨迹上施加运动学/动力学约束。 |

#### ③ 工程评测（DWB vs TEB）

| 字段 | 内容 |
|------|------|
| **代表** | Elbouhy et al. (2022) *Benchmarking local motion planners* |
| **核心价值** | TEB 突发障碍绕行更灵活；DWB 负载更平稳。 |

---

## 2. 架构

```
全局路径（稀疏位姿）
    │
    ▼
初始化弹性带（位姿序列 + 时间间隔 Δt_i）
    │
    ▼
迭代优化（g2o / Levenberg-Marquardt）
    ├── 障碍代价（点障碍 / 膨胀）
    ├── 路径跟踪代价
    ├── 时间最优代价
    ├── 速度/加速度约束
    └── 运动学约束（DiffDrive / Ackermann）
    │
    ▼
提取首段速度 → cmd_vel
```

**与 DWB 差异**：TEB 在**连续空间**优化，支持**倒车**与**多拓扑**（左绕/右绕）；DWB 在速度空间离散采样。

---

## 3. 数学原理（Step-by-Step）

### Step 1：弹性带表示

$n$ 个位姿 $s_i = (x_i, y_i, \theta_i)$，$n-1$ 个时间间隔 $\Delta t_i$：

$$
\tau = \{s_0, \Delta t_0, s_1, \Delta t_1, \ldots, s_{n-1}\}
$$

### Step 2：目标函数

$$
\min_\tau \sum_i \Big(
w_t \Delta t_i^2 +
w_o \, d_{obs}(s_i)^2 +
w_v \|v_i - v_{pref}\|^2 +
w_a \|a_i\|^2
\Big)
$$

| 项 | 含义 |
|----|------|
| $\Delta t_i^2$ | 时间最优 |
| $d_{obs}$ | 距障碍距离 |
| $v_{pref}$ | 期望速度 |

### Step 3：运动学约束（DiffDrive）

相邻位姿间速度、加速度由有限差分近似，满足非完整约束。

### Step 4：障碍表示

costmap 致命格转为**点障碍**；可用 `costmap_converter` 提取多边形。TEB **不直接**在栅格上求代价，而是距离场形式。

### Step 5：优化求解

用 g2o 图优化迭代；支持**同伦类**多轨迹并行优化（左/右绕障）。

### Step 6：控制提取

优化后取 $\tau$ 前两帧位姿差分 → $(v_x, \omega_z)$。

---

## 4. 配置要点（ROS / Nav2）

| 参数 | 说明 |
|------|------|
| `max_vel_x` | 最大线速度 |
| `acc_lim_x` | 加速度限制 |
| `min_obstacle_dist` | 最小障碍距离 |
| `weight_obstacle` | 障碍权重 |
| `weight_optimaltime` | 时间最优权重 |
| `allow_init_with_backwards_motion` | 允许倒车 |

---

## 5. 调参要点

| 现象 | 调整 |
|------|------|
| 优化不收敛 | 增大迭代次数，简化障碍模型 |
| 贴障太近 | 增大 min_obstacle_dist |
| 计算慢（80–150 ms） | 减少 band 点数，降低频率 |
| 局部最优（只左绕） | 启用 homotopy class planning |

---

## 6. 与 DWB / MPPI 对比

| 维度 | TEB | DWB | MPPI |
|------|-----|-----|------|
| 延迟 | 80–150 ms | 20–50 ms | 50–200 ms |
| 倒车 | ✅ | 可选 | ✅ |
| 动态障碍 | 强 | 强 | 强 |
| 轨迹平滑 | 中 | 中–高 | 高 |

---

## 7. Autonomy 移植清单

```
□ 评估是否需要（Phase 4）
□ 依赖 g2o 或自研优化器
□ 障碍从 Costmap2DWrapper 转点/多边形
□ 继承 ControllerInterface
```

---

## 8. 参考文献

1. Rösmann, C., Hoffmann, F., & Bertram, T. (2017). *Integrated Online Trajectory Optimization*. IEEE RAM.
2. teb_local_planner: [wiki.ros.org/teb_local_planner](http://wiki.ros.org/teb_local_planner)
