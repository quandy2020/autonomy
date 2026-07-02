# 8. 定位算法综述（Survey）

本文从**学术发展史、算法体系、工程实践、Autonomy 定位**四个维度，系统综述移动机器人定位（Localization）领域，并明确 `autonomy/localization` 的能力边界与选型依据。

> 公式推导见 [§3 数学原理](03_math.md)；实现细节见 [§5 架构](05_architecture.md)、[§6 Atlas](06_atlas.md)。

---

## 8.1 概述：定位在导航栈中的位置

| 子问题 | 典型方法 | Autonomy 模块 | 频率 |
|--------|----------|---------------|------|
| 全局定位（已知地图） | AMCL / ICP | AMCL（配置就绪） | 5–20 Hz |
| 视觉 SLAM | ORB-SLAM / VINS | **Atlas** | 15–30 Hz |
| 激光 SLAM | Cartographer / Gmapping | Cartographer（配置就绪） | 5–10 Hz |
| 融合定位 | EKF / UKF / 因子图 | 规划中 | — |

<div class="nav-costmap-banner">
  <strong>Autonomy 定位管线</strong>
  <span class="nav-costmap-detail">传感器 → Atlas/AMCL → TF(map↔odom↔base) → Map / Planning / Control</span>
  <span class="nav-costmap-arrow">Atlas 建图 → MapServer → AMCL →</span>
</div>

```
相机 ──→ Atlas (VSLAM) ──→ T_cw / 稀疏地图 ──┐
                                           ├──→ TF ──→ Planning / Control
激光 + 地图 ──→ AMCL (待集成) ──→ map→odom ─┘
激光 ──→ Cartographer (待集成) ──→ 子图 + 位姿图
```

---

## 8.2 定位问题的形式化

### 8.2.1 位姿估计

机器人在世界系中的位姿 $\mathbf{x} = (x, y, z, \phi, \theta, \psi)$ 或 2D 简化 $(x, y, \theta)$。

**静态定位**（已知地图 $\mathcal{M}$）：

$$
\hat{\mathbf{x}}_t = \arg\max_{\mathbf{x}} p(\mathbf{x} \mid z_t, \mathcal{M})
$$

**SLAM**（地图未知）：

$$
(\hat{\mathbf{x}}_{1:t}, \hat{\mathcal{M}}) = \arg\max p(\mathbf{x}_{1:t}, \mathcal{M} \mid z_{1:t}, u_{1:t})
$$

### 8.2.2 不确定性表示

| 表示 | 方法 | 特点 |
|------|------|------|
| 点估计 | PnP / ICP | 简单，无置信度 |
| 高斯 | EKF / UKF | 单峰，计算快 |
| 粒子集 | AMCL | 多峰，可全局定位 |
| 图优化 | g2o / GTSAM | 批量最优，可回环 |

---

## 8.3 算法分类

### 8.3.1 滤波方法

**扩展卡尔曼滤波（EKF-SLAM）**

$$
\mathbf{x}_t = f(\mathbf{x}_{t-1}, u_t) + \mathbf{w}, \quad \mathbf{z}_t = h(\mathbf{x}_t) + \mathbf{v}
$$

线性化后递推均值与协方差。$O(n^2)$ 路标关联，大规模地图受限。

**粒子滤波（MCL / AMCL）**

$$
p(\mathbf{x}_t \mid z_{1:t}) \approx \sum_m w_t^{(m)} \delta(\mathbf{x}_t - \mathbf{x}_t^{(m)})
$$

- 可表示多模态后验（全局定位）
- KLD 自适应粒子数
- Autonomy 配置：`config/localization/amcl/amcl.lua`

### 8.3.2 视觉方法

**特征点 SLAM（Atlas 所属）**

| 阶段 | 方法 | Atlas 实现 |
|------|------|------------|
| 前端 | ORB + 匹配 | `feature/orb_extractor` |
| 初始化 | 5pt/8pt + 三角化 | `solve/essential_solver` |
| 跟踪 | PnP + BA | `tracking_module` |
| 建图 | LBA | `mapping_module` |
| 回环 | BoW + Sim3 + GBA | `global_optimization_module` |

**直接法 / 半直接法**

最小化光度误差，无需描述子。代表：DSO、SVO。Atlas 未采用，因 ORB 在纹理变化下更鲁棒且便于回环。

**视觉-惯性（VI-SLAM）**

预积分 IMU 约束 + 视觉重投影。代表：VINS-Mono、ORB-SLAM3 VI。Atlas 当前未融合 IMU。

### 8.3.3 激光方法

**扫描匹配（Scan Matching）**

$$
\hat{T} = \arg\min_T \sum_i \| T \mathbf{p}_i - \mathcal{M}_{\text{nearest}}(T \mathbf{p}_i) \|^2
$$

Ceres / GICP 求解。Cartographer 2D 前端核心。

**子图 SLAM（Cartographer）**

- 局部 scan-to-submap 匹配
- 位姿图优化（`pose_graph.lua`）
- 纯定位模式：`pure_localization_trimmer`

Autonomy 配置：`config/localization/cartographer/`

### 8.3.4 图优化方法

统一目标：

$$
\mathbf{x}^* = \arg\min \sum_k \| \mathbf{r}_k(\mathbf{x}) \|_{\mathbf{\Omega}_k}^2
$$

| 因子类型 | 约束 | Atlas |
|----------|------|-------|
| 重投影 | 2D ↔ 3D | g2o reproj edge |
| 里程计 | 相对位姿 | spanning tree |
| 回环 | Sim3 | loop_detector |
| IMU | 预积分 | 未实现 |
| GPS | 绝对位姿 | 未实现 |

---

## 8.4 传感器与算法匹配

| 传感器组合 | 推荐算法 | Autonomy 状态 |
|------------|----------|---------------|
| 单目相机 | 特征 SLAM | Atlas ✓ |
| 双目相机 | 特征 SLAM / VI | Atlas ✓ |
| RGB-D | 特征 SLAM / ICP | Atlas ✓ |
| 2D 激光 + 地图 | AMCL | 配置 ✓ |
| 2D 激光 SLAM | Cartographer / Gmapping | 配置 ✓ |
| 3D 激光 | Cartographer 3D / LOAM | 配置 ✓ |
| 相机 + IMU | VI-SLAM | 待扩展 |
| GPS + 激光 | 融合因子图 | 待扩展 |

---

## 8.5 经典系统对比

| 系统 | 传感器 | 后端 | 回环 | 许可证 |
|------|--------|------|------|--------|
| **Atlas** | 单/双/RGB-D | g2o | BoW | Apache 2.0 |
| ORB-SLAM3 | + IMU | g2o | DBoW2 | GPL-3.0 |
| stella_vslam | 单/双/RGB-D | g2o | BoW | MIT |
| nav2_amcl | 2D 激光 | 粒子滤波 | — | Apache 2.0 |
| Cartographer | 2D/3D 激光 | 位姿图 | 子图 | Apache 2.0 |
| LIO-SAM | 激光+IMU | GTSAM | — | BSD |
| VINS-Mono | 单目+IMU | Ceres | 4-DOF | GPL-3.0 |

Autonomy 选择 Atlas  lineage 的原因：Apache 2.0 许可、成熟 ORB 管线、g2o 生态、与 stella_vslam 架构兼容。

---

## 8.6 精度与漂移

### 8.6.1 误差来源

| 来源 | 视觉 SLAM | 激光 AMCL |
|------|-----------|-----------|
| 传感器噪声 | 特征匹配、标定 | 激光角分辨率 |
| 算法近似 | 线性化、RANSAC | 粒子数不足 |
| 累积漂移 | 无回环时显著 | 里程计累积 |
| 环境 | 纹理弱、动态物体 | 对称走廊 |

### 8.6.2 漂移量级（典型）

| 方法 | 无回环 | 有回环 |
|------|--------|--------|
| 单目 VSLAM | 1–5% 路径长度 | < 0.1% |
| 双目/RGB-D | 0.5–2% | < 0.05% |
| AMCL（已知地图） | 无累积（有界） | — |
| Cartographer | 0.1–1% | 子图回环 |

---

## 8.7 工程选型决策树

```
需要 3D / 视觉？
├── 是 → 有 IMU？
│       ├── 是 → [待扩展] VI-SLAM
│       └── 否 → Atlas (单/双/RGB-D)
└── 否 → 有先验 2D 地图？
        ├── 是 → AMCL (激光 + 里程计)
        └── 否 → Cartographer (激光 SLAM)
```

---

## 8.8 Autonomy 定位能力矩阵

| 能力 | Atlas | AMCL | Cartographer |
|------|-------|------|--------------|
| 代码实现 | ✓ | 配置 | 配置 |
| 实时跟踪 | ✓ | 预期 ✓ | 预期 ✓ |
| 建图 | ✓ 稀疏 3D | ✗ | 预期 ✓ 2D/3D |
| 回环 | ✓ BoW | ✗ | 预期 ✓ |
| 纯定位 | ✓ load map | 预期 ✓ | 预期 ✓ |
| TF 输出 | 需桥接 | 预期原生 | 预期原生 |
| 与 Map 集成 | 导出地图 | 消费 /map | 发布 /map |

---

## 8.9 发展趋势

| 方向 | 说明 | Autonomy 路线 |
|------|------|---------------|
| 多传感器融合 | 视觉+激光+IMU 因子图 | 统一 LocalizationInterface |
| 语义 SLAM | 物体级路标 | Atlas marker 已有基础 |
| 终身 SLAM | 地图更新与遗忘 | local_map_cleaner |
| 学习特征 | SuperPoint / NetVLAD | 可替换 ORB 前端 |
| 云定位 | 全局地图匹配 | 远期 |

---

## 8.10 参考文献

1. [Mur-Artal, R. et al. **ORB-SLAM2**. IEEE TRO, 2017.](https://doi.org/10.1109/TRO.2017.2705103) / [Campos, C. et al. **ORB-SLAM3**. IEEE TRO, 2021.](https://doi.org/10.1109/TRO.2021.3075644)
2. [Engel, J. et al. **DSO: Direct Sparse Odometry**. ECCV, 2016.](https://doi.org/10.1007/978-3-319-46493-0_35)
3. [Hess, W. et al. **Real-Time Loop Closure in 2D LIDAR SLAM**. ICRA, 2016.](https://research.google/pubs/real-time-loop-closure-in-2d-lidar-slam/)
4. [Fox, D. et al. **Monte Carlo Localization: Efficient Position Estimation for Mobile Robots**. AAAI, 1999.](https://cdn.aaai.org/AAAI/1999/AAAI99-050.pdf)
5. [Kümmerle, R. et al. **g2o: A General Framework for Graph Optimization**. ICRA, 2011.](https://doi.org/10.1109/ICRA.2011.5979949)
6. [stella_vslam / stb_vslam 开源实现文档](https://stella-cv.readthedocs.io/)
7. [Navigation2 **nav2_amcl** 官方文档](https://docs.nav2.org/configuration/packages/configuring-amcl.html)

---

## 8.11 延伸阅读

| 主题 | Autonomy 文档 |
|------|---------------|
| Atlas 实现 | [06_atlas.md](06_atlas.md) |
| 数学公式 | [03_math.md](03_math.md) |
| AMCL 配置 | [07_amcl.md](07_amcl.md) |
| 架构设计 | [05_architecture.md](05_architecture.md) |
| 地图模块 | [../07_Map/08_survey.md](../07_Map/08_survey.md) |
| 路径规划 | [../08_Planning/09_survey.md](../08_Planning/09_survey.md) |
