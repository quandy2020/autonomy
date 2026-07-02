# 7. AMCL 粒子滤波定位

**Adaptive Monte Carlo Localization (AMCL)** 是 ROS 导航栈中最常用的 **2D 激光定位** 算法，Autonomy 在 `config/localization/amcl/amcl.lua` 中预留了与 **nav2_amcl** 对齐的配置项。

> **当前状态**：配置与 Proto 定义已就绪，C++ 实现尚未合入 `autonomy/localization`。本章描述算法原理、配置语义与预期集成方式。数学细节见 [§3.10 AMCL 粒子滤波](03_math.md#310-amcl-粒子滤波)。

---

## 7.1 算法概述

AMCL 用粒子滤波估计机器人在**已知 2D 占据栅格地图**中的位姿 $(x, y, \theta)$：

$$
p(\mathbf{x}_t \mid z_{1:t}, u_{1:t}) \approx \sum_{m=1}^{M} w_t^{(m)} \delta(\mathbf{x}_t - \mathbf{x}_t^{(m)})
$$

| 步骤 | 操作 | 输入 |
|------|------|------|
| 预测 | 运动模型传播粒子 | 里程计 $u_t$ |
| 更新 | 激光似然加权 | LaserScan $z_t$ + 占据栅格 |
| 重采样 | KLD 自适应粒子数 | $N_{\text{eff}}$ |
| 输出 | 加权位姿 + TF | `map→odom` |

---

## 7.2 配置参数详解

配置文件：`config/localization/amcl/amcl.lua`

### 7.2.1 运动模型

| 参数 | 默认 | 含义 |
|------|------|------|
| `alpha1` | 0.2 | 旋转→旋转噪声系数 |
| `alpha2` | 0.2 | 平移→旋转噪声系数 |
| `alpha3` | 0.2 | 平移→平移噪声系数 |
| `alpha4` | 0.2 | 旋转→平移噪声系数 |
| `alpha5` | 0.2 | 全向机器人平移噪声 |
| `robot_model_type` | 2 | 0=差速, 1=全向, 2=nav2 DifferentialMotionModel |

差速驱动运动分解（sample_motion_odometry）：

$$
\delta_{\text{rot1}} = \mathrm{angle}(\delta, q_{\mathrm{last}}), \quad
\delta_{\text{trans}} = \|\delta\|, \quad
\delta_{\text{rot2}} = \theta_{\text{new}} - \theta_{\text{old}} - \delta_{\text{rot1}}
$$

其中 $q_{\mathrm{last}}$ 为 `last_pose`。

### 7.2.2 粒子滤波

| 参数 | 默认 | 含义 |
|------|------|------|
| `min_particles` | 500 | 最少粒子数 |
| `max_particles` | 2000 | 最多粒子数 |
| `pf_err` | 0.01 | KLD 采样误差界 |
| `pf_z` | 0.99 | KLD 置信度 |
| `recovery_alpha_fast` | 0.0 | 快速恢复（ kidnapping） |
| `recovery_alpha_slow` | 0.0 | 慢速恢复 |
| `resample_interval` | 1 | 重采样间隔（帧） |

**KLD 自适应采样**：根据粒子占据的网格数动态调整 $M$，在精度与算力间平衡。

### 7.2.3 激光模型

| 参数 | 默认 | 含义 |
|------|------|------|
| `laser_model_type` | 2 | 0=Beam, 1=LikelihoodField, 2=LikelihoodFieldProb |
| `max_beams` | 60 | 每次更新使用的光束数 |
| `laser_likelihood_max_dist` | 2.0 m | 距离场有效范围 |
| `z_hit` | 0.95 | 命中权重 |
| `z_rand` | 0.05 | 随机测量权重 |
| `z_max` | 0.05 | 最大量程权重 |
| `z_short` | 0.1 | 短距权重 |
| `sigma_hit` | 0.2 m | 命中高斯标准差 |
| `lambda_short` | 0.1 | 短距指数衰减 |

**Likelihood Field Prob** 模型（`laser_model_type = 2`）在 Likelihood Field 基础上增加 beam skipping，对动态障碍更鲁棒。

### 7.2.4 坐标系与话题

| 参数 | 默认 | 含义 |
|------|------|------|
| `global_frame_id` | `"map"` | 全局坐标系 |
| `odom_frame_id` | `"odom"` | 里程计坐标系 |
| `base_frame_id` | `"base_link"` | 机器人基座 |
| `scan_topic` | `"/scan"` | 激光话题 |
| `map_topic` | `"/map"` | 静态地图话题 |
| `tf_broadcast` | true | 发布 map→odom |
| `transform_tolerance` | 1.0 s | TF 容差 |

### 7.2.5 更新阈值

| 参数 | 默认 | 含义 |
|------|------|------|
| `update_min_a` | 0.2 rad | 最小角度变化触发更新 |
| `update_min_d` | 0.25 m | 最小平移变化触发更新 |

避免机器人静止时无效更新，降低 CPU 占用。

### 7.2.6 初始位姿

```lua
set_initial_pose = false,
initial_pose = { x = 0.0, y = 0.0, z = 0.0, yaw = 0.0 },
always_reset_initial_pose = false,
```

或通过 RViz「2D Pose Estimate」注入高斯先验粒子群。

---

## 7.3 预期集成架构

```
┌──────────────┐     ┌─────────────┐     ┌──────────────┐
│  MapServer   │────►│    AMCL     │◄────│   /scan      │
│  /map        │     │  (待实现)    │◄────│   /odom      │
└──────────────┘     └──────┬──────┘     └──────────────┘
                            │
                            ▼ TF map→odom
                     ┌──────────────┐
                     │  Planning    │
                     │  Control     │
                     └──────────────┘
```

### 7.3.1 与 nav2_amcl 对齐项

| 能力 | nav2_amcl | Autonomy 规划 |
|------|-----------|---------------|
| 差速/全向运动模型 | ✓ | `robot_model_type` |
| Likelihood Field Prob | ✓ | `laser_model_type = 2` |
| KLD 自适应粒子 | ✓ | min/max_particles |
| Beam skipping | ✓ | do_beamskip |
| 初始位姿 | ✓ | set_initial_pose |

---

## 7.4 使用场景

| 场景 | 适用性 | 说明 |
|------|--------|------|
| 已知 2D 地图室内导航 | ★★★★★ | 标准用法 |
| 动态环境 | ★★★☆☆ | LikelihoodFieldProb + beam skipping |
| 大场景 | ★★★☆☆ | 增大 max_particles |
| 无激光 / 纯视觉 | ✗ | 使用 Atlas |
| 3D 地形 | ✗ | 使用 Cartographer 3D |

---

## 7.5 调参建议

| 问题 | 调整 |
|------|------|
| 定位发散 | 增大 `min_particles`；检查地图与激光对齐 |
| CPU 过高 | 减小 `max_beams`；增大 `update_min_d/a` |
|  kidnapped | 设置 `recovery_alpha_fast/slow` > 0 |
| 旋转漂移 | 增大 `alpha1/alpha4` |
| 平移漂移 | 增大 `alpha3` |
| 激光噪点 | 减小 `z_rand`；增大 `sigma_hit` |

---

## 7.6 与 Atlas 的关系

| 维度 | AMCL | Atlas |
|------|------|-------|
| 地图类型 | 2D 占据栅格 | 稀疏 3D 路标 |
| 传感器 | 2D 激光 | 相机 |
| 输出维度 | SE(2) | SE(3) |
| 先验地图 | 必须 | 可选（可 SLAM 建图） |

二者可互补：Atlas 建图 → 导出 2D 占据栅格 → MapServer → AMCL 定位。

---

## 7.7 启用配置（集成后）

```lua
-- config/localization/localization.lua
AUTONOMY_LOCALIZATION = {
    default_algorithm = "amcl",
    enabled = true,
    amcl = AMCL_OPTIONS,
}
```

```lua
-- config/autonomy.lua
include "localization/localization.lua"
AUTONOMY = { localization = AUTONOMY_LOCALIZATION }
```

集成完成前，请使用 [Atlas](06_atlas.md) 作为定位方案。
