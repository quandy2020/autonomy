# 5. 地图表示综述（Survey）

> **本文范围**：地图表示谱系、近三十年里程碑、Autonomy 能力边界与**选型依据**。  
> 公式与图层实现见 [§3 Costmap2D](03_costmap2d.md) · [§4 GridMap](04_grid_map.md)；架构见 [§1](01_architecture.md)。

---

## 5.1 综述定位

| 维度 | 本文 | 其他文档 |
|------|------|----------|
| 坐标变换、膨胀公式 | 不展开 | [§3.5–§3.8](03_costmap2d.md#35-坐标变换) · [§4.6–§4.8](04_grid_map.md#46-坐标变换) |
| 图层配置、API 时序 | 不展开 | [§0 指南](00_guide.md) · [§1](01_architecture.md) |
| 表示分类、历史、前沿 | **本文** | — |
| Autonomy 内置能力 | **本文 §5.11** | [§2 组件总览](02_map_components.md) |

**建议阅读顺序**

| 角色 | 路径 |
|------|------|
| 选型 / 集成 | §5.11 → [§2](02_map_components.md) → [§0.7 排错](00_guide.md#07-故障排查) |
| 算法研发 | §5.3 分类 → §5.4 时间轴 → §5.5 表示族 → §3–§4 专题 |
| 背景调研 | §5.4 → §5.5 → §5.13 前沿 → §5.14 参考文献 |

移动机器人导航依赖**环境地图**作为规划与感知的空间基准：

| 子问题 | 典型地图类型 | Autonomy 模块 | 频率 |
|--------|--------------|---------------|------|
| 全局定位 | 静态占据栅格 | `MapServer` | 一次性 / 低频 |
| 路径规划 | 2D 代价地图 | `Costmap2D` | 1–10 Hz |
| 局部避障 | 滚动代价地图 | `Costmap2D` (rolling) | 5–20 Hz |
| 地形感知 | 2.5D 多层栅格 | `GridMap` | 1–10 Hz |
| SLAM 建图 | 点云 / 子图 | 外部模块 → 注入 | 变化 |

<div class="nav-costmap-banner">
  <strong>Autonomy 地图管线</strong>
  <span class="nav-costmap-detail">SLAM → MapServer → static_layer → obstacle_layer → inflation_layer → Planning</span>
  <span class="nav-costmap-arrow">GridMap 并行独立 →</span>
</div>

```
SLAM / 文件 ──→ MapServer ──→ OccupancyGrid ──→ static_layer ──┐
激光 / 点云 ────────────────────────────────→ obstacle_layer ──┼──→ Costmap2D ──→ Planning
                                                              │
                                              inflation_layer ─┘

深度 / 点云 ──→ GridMap (elevation / slope / ...) ──→ [未来桥接]
```

---

## 5.2 地图表示的形式化

### 5.2.1 占据栅格（Occupancy Grid）

环境 $\mathcal{W} \subset \mathbb{R}^2$ 离散为 $N_x \times N_y$ 栅格，每格存储占据概率或标签：

$$
p(m_{ij} = \mathrm{occupied} \mid z_{1:t})
$$

典型输出：$o_{ij} \in \{-1, 0, \ldots, 100\}$（未知 / 自由 / 占据百分比）。

**Autonomy 实现**：`MapServer` 加载 YAML+PGM，`StaticLayer` 消费 `OccupancyGrid`。

### 5.2.2 代价地图（Costmap）

在占据栅格基础上引入**通行代价** $c_{ij} \in [0, 255]$，使规划器不仅避开障碍，还能**远离障碍**：

$$
c_{ij} = f(d_{ij}^{\text{obs}}, r_i, \lambda)
$$

其中 $d_{ij}^{\text{obs}}$ 为到最近障碍的距离，$r_i$ 为机器人内切半径，$\lambda$ 为衰减系数。

**Autonomy 实现**：`Costmap2D` + `InflationLayer`，指数衰减公式见 [§3.8](03_costmap2d.md#38-inflationlayer-详解)。

### 5.2.3 2.5D 多层栅格（Grid Map）

在 XY 平面维护多层浮点属性 $\{L_k\}$，第三维信息编码在层值中：

$$
\mathcal{M}_{2.5D} = \{ L_k(i,j) \mid k = 1,\ldots,K \}
$$

典型层：高程 $z$、方差 $\sigma_z^2$、坡度、法向量、可通行性。

**Autonomy 实现**：`grid_map::GridMap`，ETH `grid_map` 架构。

### 5.2.4 其他表示（Autonomy 未内置）

| 表示 | 特点 | 典型应用 |
|------|------|----------|
| 点云地图 | 精确 3D，数据量大 | LOAM、LIO-SAM |
| 八叉树 (OctoMap) | 稀疏 3D 占据 | 无人机、机械臂 |
| 拓扑地图 | 节点+边，全局结构 | 多楼层、大范围 |
| 语义 / 场景图 | 物体、房间、关系 | 任务规划、人机交互 |
| ESDF / TSDF | 有符号距离场 | Voxblox、实时避障 |
| 神经隐式 (NeRF/3DGS) | 连续场、可微渲染 | 稠密 SLAM、仿真 |

---

## 5.3 表示分类全景

近年综述将机器人空间记忆按**几何—语义—拓扑**逐层叠加（metric → metric–semantic → metric–semantic–topological）。Autonomy 当前落在**度量 2D/2.5D**层，语义与拓扑需外部模块注入。

### 5.3.1 五维分类法

```
环境地图表示
├── 几何粒度：度量（栅格/点云/体素）· 拓扑（图）· 混合（SSH / 场景图）
├── 空间维度：2D · 2.5D · 3D · 隐式连续场
├── 语义层级：几何 only · 物体实例 · 房间/功能 · 开放词汇
├── 更新方式：静态 · 增量 SLAM · 滚动局部 · 动态物体剔除
└── 存储结构：稠密数组 · 稀疏树 · 子图 · 神经网络权重
```

### 5.3.2 方法—特性矩阵

| 表示族 | 代表 | 定位精度 | 规划直接可用 | 内存缩放 | 长期运维 | Autonomy |
|--------|------|----------|--------------|----------|----------|----------|
| 2D 占据栅格 | Elfes, Nav2 map_server | 中 | ✅ 经 costmap | $O(N^2)$ | 静态优 | ✅ MapServer |
| 2D 代价图 | costmap_2d | 中 | ✅ | $O(N^2)$ | 动态融合 | ✅ Costmap2D |
| 2.5D 多层栅格 | grid_map | 中–高 | 需桥接 | $O(KN^2)$ | 地形优 | ✅ GridMap |
| 3D 八叉树 | OctoMap | 高 | 需投影/3D 规划 | 稀疏 | 探索优 | ❌ |
| 子图 / 图优化 | Cartographer, g2o | 高 | 导出 2D | 子图数 | 大规模 SLAM | 外部注入 |
| 拓扑 / 外观 | FAB-MAP, SSH | 低–中 | 全局路由 | $O(\text{places})$ | 回环检测 | ❌ |
| 度量–语义 | Kimera, SuMa++ | 高 | 加权代价 | 中–大 | 任务语义 | ❌ |
| 3D 场景图 | Hydra, 3D Scene Graph | 高 | 高层规划 | 分层图 | 长期语义 | ❌ |
| 神经隐式 | NeRF-SLAM, 3DGS-SLAM | 高 | 研究中 | 权重体积 | 渲染/重建 | ❌ |

---

## 5.4 发展时间轴（1995–2025）

按五个历史阶段分块展示里程碑；完整对照见 [§5.4.1](#541-分阶段特征表)。

<div class="planning-timeline-v2">

<div class="timeline-era-block era-foundation">
  <div class="timeline-era-header">概率度量建图 · 1995–2004</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1996</div>
      <div class="timeline-milestone-title">Thrun 占据栅格综述</div>
      <div class="timeline-milestone-desc">贝叶斯逆传感器模型，概率地图更新框架</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1998</div>
      <div class="timeline-milestone-title">FastSLAM</div>
      <div class="timeline-milestone-desc">Rao-Blackwellized 粒子滤波，$O(M\log K)$ 地标</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2001</div>
      <div class="timeline-milestone-title">Topological SLAM</div>
      <div class="timeline-milestone-desc">Choset & Nagatani，拓扑+度量混合</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2004</div>
      <div class="timeline-milestone-title">Hybrid SSH</div>
      <div class="timeline-milestone-desc">Kuipers，局部度量+全局拓扑</div>
    </div>
  </div>
</div>

<div class="timeline-era-block era-reactive">
  <div class="timeline-era-header">图优化与 3D 体素 · 2005–2014</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2008</div>
      <div class="timeline-milestone-title">FAB-MAP</div>
      <div class="timeline-milestone-desc">外观拓扑 SLAM，在线回环检测</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2010</div>
      <div class="timeline-milestone-title">g2o / 图 SLAM</div>
      <div class="timeline-milestone-desc">Grisetti 等，位姿图非线性优化标准框架</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2013</div>
      <div class="timeline-milestone-title">OctoMap · grid_map</div>
      <div class="timeline-milestone-desc">3D 概率八叉树；ETH 2.5D 多层栅格</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2014</div>
      <div class="timeline-milestone-title">NDT-OM · ElasticFusion</div>
      <div class="timeline-milestone-desc">正态分布变换占据；TSDF 稠密 RGB-D SLAM</div>
    </div>
  </div>
</div>

<div class="timeline-era-block era-reactive">
  <div class="timeline-era-header">工程栈与语义萌芽 · 2015–2019</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2015</div>
      <div class="timeline-milestone-title">ORB-SLAM · Voxblox</div>
      <div class="timeline-milestone-desc">特征视觉 SLAM；ESDF/TSDF 实时体素融合</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2016</div>
      <div class="timeline-milestone-title">Cartographer</div>
      <div class="timeline-milestone-desc">子图 + 扫描匹配，产品级 2D/3D SLAM</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2017</div>
      <div class="timeline-milestone-title">SemanticFusion</div>
      <div class="timeline-milestone-desc">CNN 语义注入 TSDF 体素地图</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2019</div>
      <div class="timeline-milestone-title">3D Scene Graph</div>
      <div class="timeline-milestone-desc">Armeni 等，统一语义+几何+相机</div>
    </div>
  </div>
</div>

<div class="timeline-era-block era-modern">
  <div class="timeline-era-header">度量–语义与神经隐式 · 2020–2023</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2020</div>
      <div class="timeline-milestone-title">NeRF · Kimera</div>
      <div class="timeline-milestone-desc">神经辐射场；实时度量–语义 SLAM</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2022</div>
      <div class="timeline-milestone-title">Hydra</div>
      <div class="timeline-milestone-desc">增量构建 3D 场景图，房间级拓扑</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2023</div>
      <div class="timeline-milestone-title">3D Gaussian Splatting</div>
      <div class="timeline-milestone-desc">显式高斯原语，实时渲染与 SLAM 融合</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2023</div>
      <div class="timeline-milestone-title">NeRF-SLAM / GS-SLAM</div>
      <div class="timeline-milestone-desc">隐式/高斯 SLAM，稠密重建+定位</div>
    </div>
  </div>
</div>

<div class="timeline-era-block era-modern">
  <div class="timeline-era-header">开放语义与动态场景 · 2024–2025</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2024</div>
      <div class="timeline-milestone-title">Khronos</div>
      <div class="timeline-milestone-desc">动态环境时空度量–语义 SLAM</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2024</div>
      <div class="timeline-milestone-title">SNI-SLAM · SemGauss-SLAM</div>
      <div class="timeline-milestone-desc">语义神经隐式 / 语义高斯 SLAM</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2024</div>
      <div class="timeline-milestone-title">开放词汇地图</div>
      <div class="timeline-milestone-desc">CLIP 类特征，自然语言查询地标</div>
    </div>
    <div class="timeline-milestone timeline-milestone-highlight">
      <div class="timeline-milestone-year">2025</div>
      <div class="timeline-milestone-title">Autonomy map</div>
      <div class="timeline-milestone-desc">Nav2 costmap + ETH grid_map 工程融合</div>
    </div>
  </div>
</div>

</div>

### 5.4.1 分阶段特征表

| 阶段 | 年代 | 代表方法 | 核心突破 | 局限 |
|------|------|----------|----------|------|
| 概率栅格 | 1985–1999 | Occupancy Grid, EKF-SLAM | 逆传感器模型、在线更新 | $O(K^2)$ 地标、栅格内存 |
| 粒子与子图 | 2000–2014 | FastSLAM, Cartographer, OctoMap | 可扩展 SLAM、3D 稀疏体素 | 调参复杂、3D 规划难 |
| 图优化工程化 | 2010–2018 | g2o, costmap_2d, Nav2 | 非线性优化、插件化代价图 | 仍以 2D 规划为主 |
| 语义与场景图 | 2017–2023 | Kimera, Hydra, 3D Scene Graph | 物体/房间/关系可查询 | 算力与标注成本 |
| 神经隐式 | 2020–至今 | NeRF, 3DGS, 语义 NeRF | 连续场、照片级重建 | 实时性、可编辑性、安全认证 |

---

## 5.5 表示族速览

以下各节为**摘要**；Autonomy 已实现类型见 §5.11，完整推导见 [§3](03_costmap2d.md) · [§4](04_grid_map.md)。

### 5.5.1 2D 度量栅格与代价图

**占据栅格**（Elfes 1989；Thrun 1996 概率框架）将环境离散为独立单元，用 log-odds 或 Beta 分布融合激光/声纳。**代价图**在占据基础上叠加距离相关代价，Nav2 `costmap_2d`（2008–）以插件链（static / obstacle / inflation）成为移动底盘事实标准。

| 里程碑 | 年份 | 贡献 |
|--------|------|------|
| [Elfes, occupancy grids](https://doi.org/10.1109/2.30720) | 1989 | 栅格感知与导航 |
| [Thrun, probabilistic mapping](https://robots.stanford.edu/papers/thrun.mapping-tr.pdf) | 1996 | 贝叶斯占据更新 |
| nav2_costmap_2d | 2008+ | 多层融合、滚动窗口 |

Autonomy：`MapServer` + `Costmap2DWrapper`。

### 5.5.2 3D 体素与距离场

**OctoMap**（Hornung et al., 2013）用八叉树存概率占据，显式建模 free / occupied / unknown。**Voxblox**（Oleynikova et al., 2017）在 TSDF/ESDF 上支持规划器梯度查询，优于 costmap 指数膨胀近似。

| 里程碑 | 年份 | 贡献 |
|--------|------|------|
| [OctoMap](https://doi.org/10.1007/s10514-012-9321-0) | 2013 | 稀疏 3D 概率地图 |
| [Voxblox](https://doi.org/10.1109/LRA.2017.2659858) | 2017 | 实时 ESDF/TSDF 融合 |
| [NDT-OM](https://ieeexplore.ieee.org/document/6630807) | 2013 | 大场景 NDT 占据图 |

Autonomy：`VoxelLayer` 将 3D 投影到 2D；完整 OctoMap/ESDF 未内置。

### 5.5.3 2.5D 多层栅格

**grid_map**（Fankhauser et al., 2014；ETH 2013 起）在固定分辨率 XY 栅格上堆叠命名浮点层，用循环 buffer 实现 $O(1)$ 滑动，适合高程、坡度、方差等地形量。

| 里程碑 | 年份 | 贡献 |
|--------|------|------|
| [grid_map library](https://doi.org/10.1109/IROS.2014.6942672) | 2014 | 2.5D 多层 + 插值 |
| [Traversability analysis](https://doi.org/10.1109/IROS.2014.6942672) | 2014 | 坡度/台阶可通行性 |

Autonomy：`GridMapWrapper`；与 Costmap 桥接待恢复。

### 5.5.4 拓扑与混合地图

**Spatial Semantic Hierarchy**（Kuipers, 2000；混合扩展 2004）在局部用度量 LPM、全局用拓扑 place 图，解决大尺度歧义。**FAB-MAP**（Cummins & Newman, 2008）用外观词袋概率模型做 place recognition，复杂度线性于地点数。

| 里程碑 | 年份 | 贡献 |
|--------|------|------|
| [SSH](https://doi.org/10.1016/S0004-3702(00)00017-5) | 2000 | 分层空间语义 |
| [Hybrid SSH](https://doi.org/10.1109/ROBOT.2004.1302485) | 2004 | 局部度量+全局拓扑 |
| [FAB-MAP](https://doi.org/10.1177/0278364907086282) | 2008 | 外观拓扑 SLAM |

Autonomy 未内置；大规模园区可外部建拓扑图，局部仍用 Costmap2D。

### 5.5.5 度量–语义与 3D 场景图

**Kimera**（Rosinol et al., 2020）实时输出 mesh + 语义标签 + 位姿图。**Hydra**（Hughes et al., 2022；IJRR 2024 扩展）增量构建**分层 3D 场景图**：体素 → 物体 → 房间 → 建筑，支持回环优化。

| 里程碑 | 年份 | 贡献 |
|--------|------|------|
| [SemanticFusion](https://doi.org/10.1109/ICRA.2017.7989593) | 2017 | 稠密语义 TSDF |
| [Kimera](https://doi.org/10.1109/ICRA.2020.9197539) | 2020 | 开源度量–语义 SLAM |
| [Hydra](https://doi.org/10.1177/02783649241229725) | 2022–2024 | 实时 3D 场景图 |
| [Khronos](https://www.roboticsproceedings.org/rss20/p081.pdf) | 2024 | 动态环境时空语义 |

语义信息可通过 GridMap 自定义层或 costmap filter 间接注入；物体级场景图需外部管线。

### 5.5.6 神经隐式与 3D 高斯

**NeRF**（Mildenhall et al., 2020）用 MLP 编码辐射场，实现视角合成。**3D Gaussian Splatting**（Kerbl et al., 2023）用显式高斯原语达到实时渲染，催生 GS-SLAM、SplaTAM、语义高斯 SLAM 等。

| 里程碑 | 年份 | 贡献 |
|--------|------|------|
| [NeRF](https://doi.org/10.1007/978-3-030-58452-8_24) | 2020 | 隐式连续场景表示 |
| [NeRF-SLAM](https://doi.org/10.1109/IROS.2023.10341674) | 2023 | 单目稠密 SLAM |
| [3D Gaussian Splatting](https://doi.org/10.1145/3592433) | 2023 | 显式原语、实时渲染 |
| [Gaussian Splatting SLAM](https://doi.org/10.1109/CVPR.2024.01842) | 2024 | 稠密 RGB-D SLAM |

前沿方向是**可查询、可编辑、可规划**的神经地图，而非仅渲染；与 Autonomy 栅格栈的桥接尚在研究阶段。

---

## 5.6 地图表示对比

| 维度 | OccupancyGrid | Costmap2D | GridMap 2.5D | OctoMap | 场景图 | NeRF/3DGS |
|------|---------------|-----------|--------------|---------|--------|-----------|
| 维度 | 2D | 2D | 2.5D | 3D | 分层 3D | 3D 隐式/显式 |
| 数值类型 | int8 | uint8 | float 多层 | float 概率 | 图+属性 | 网络/高斯 |
| 更新方式 | 静态/SLAM | 插件融合 | 应用写入 | 概率射线 | 增量构建 | 优化拟合 |
| 规划用途 | 静态障碍 | 全局/局部 | 地形 | 3D 避障 | 任务级 | 研究中 |
| 内存 | $O(N^2)$ | $O(N^2)$ | $O(KN^2)$ | 稀疏 | $O(\text{obj}+\text{places})$ | 中–大 |
| Autonomy | ✅ | ✅ | ✅ | ❌ | ❌ | ❌ |

---

## 5.7 Costmap2D 图层体系

Autonomy 采用 Nav2 风格的**插件化图层**架构：

| 图层 | 输入 | 输出 | 合并模式 |
|------|------|------|----------|
| `static_layer` | OccupancyGrid | LETHAL / FREE | Max |
| `obstacle_layer` | LaserScan / PointCloud2 | LETHAL / FREE | Max |
| `inflation_layer` | 下层 lethal 种子 | 1–253 梯度 | Max |
| `voxel_layer` | 3D 点云 | 2D 投影 | Max |
| `denoise_layer` | 连通域过滤 | 去除小簇 | Overwrite |
| `keepout_filter` | Filter mask | 禁行区 | Filter |

更新流程：各层 `updateBounds` 累加矩形 → `resetMap` → 按序 `updateCosts` → filter 后处理。

详见 [Costmap2D 文档](03_costmap2d.md#39-图层详解)。

---

## 5.8 膨胀算法的物理意义

膨胀层将离散障碍转化为**连续代价场**，使规划器自然产生安全裕度：

$$
c(d) = \lfloor 252 \cdot e^{-\lambda (d \cdot \Delta - r_i)} \rfloor
$$

| 参数 | 物理含义 | 调大效果 |
|------|----------|----------|
| $r_i$ (inscribed_radius) | 机器人内切圆 | 更大不可进入区 |
| $R$ (inflation_radius) | 膨胀传播范围 | 更远仍保持高代价 |
| $\lambda$ (cost_scaling_factor) | 衰减速度 | 更快恢复自由 |

**与 ESDF 的关系**：膨胀代价是 ESDF 的指数近似，计算复杂度 $O(N \cdot R_{\text{cell}})$ vs ESDF 的 $O(N)$，但实现简单、与 Nav2 生态兼容。

---

## 5.9 GridMap 2.5D 的设计哲学

ETH `grid_map` 的设计目标：

1. **多层解耦**：不同传感器/算法写入不同层，互不干扰
2. **循环 buffer**：$O(1)$ 滑动窗口，无需全图 copy
3. **连续插值**：支持 sub-cell 精度查询
4. **Eigen 集成**：矩阵运算、滤波器无缝对接

**2.5D vs 3D**：不维护完整 3D 体素，而是用层值编码高度信息，在计算效率与表达能力间折中。

---

## 5.10 Autonomy map 模块定位

| 能力 | 状态 | 说明 |
|------|------|------|
| 静态地图服务 | ✅ 已实现 | `MapServer`，YAML+PGM |
| 2D 代价地图 | ✅ 已实现 | Nav2 兼容，插件化图层 |
| 2.5D GridMap | ✅ 已实现 | ETH grid_map 封装 |
| Costmap ↔ GridMap 桥接 | ❌ 未实现 | `grid_map_costmap_2d/` 已注释 |
| 3D 体素地图 | 部分 | `VoxelLayer` 投影到 2D |
| 语义 / 场景图 | ❌ | 需外部模块 |

**推荐选型**：

| 场景 | 推荐组件 |
|------|----------|
| 室内平面导航 | Costmap2D（static + obstacle + inflation） |
| 已知静态环境 | MapServer + static_layer |
| 动态障碍环境 | + obstacle_layer + 激光 |
| 越野 / 台阶 | GridMap（elevation + slope） |
| 窄通道 / 精确定位 | 减小 inflation_radius，禁用 denoise |

完整决策树见 [§2.3](02_map_components.md#23-选型速查)。

---

## 5.11 与 Planning 的耦合

Costmap2D 是 Planning 模块的**核心输入**：

```
Costmap2D (char map)
    │
    ├── NavFn: F_ij = 50 + 0.8 · c_ij → 导航势场
    ├── Dijkstra: 同上，强制 Dijkstra 传播
    └── Theta*: c ≥ 253 阻塞，LOS 检测
```

规划时复制 costmap 快照，避免与地图更新线程竞争。详见 [Planning 架构](../08_Planning/01_architecture.md)。

---

## 5.12 影响因素与调参

| 因素 | 影响 | 调参方向 |
|------|------|----------|
| 分辨率 $\Delta$ | 精度 vs 内存/速度 | 室内 0.05 m，室外 0.1–0.2 m |
| 膨胀半径 $R$ | 安全裕度 vs 窄通道 | 0.35–0.55 m |
| 更新频率 | 动态响应 vs CPU | 5 Hz 通常足够 |
| Rolling window | 大地图内存 | 局部规划启用 |
| denoise | 噪声过滤 vs 薄墙保留 | 谨慎启用 |
| footprint | 碰撞模型精度 | 多边形优于圆形 |

---

## 5.13 前沿方向

| 方向 | 代表工作 | 与 Autonomy 关系 |
|------|----------|------------------|
| **ESDF / TSDF 图层** | Voxblox, FIESTA | 可替代 inflation 指数近似，支持 CHOMP/梯度规划 |
| **grid_map ↔ costmap 桥接** | grid_map_costmap_2d | 代码已预留，elevation→traversability→cost |
| **动态物体语义剔除** | Khronos, DS-SLAM | 减少动态障碍假阳性 |
| **3D 场景图** | Hydra, ConceptGraphs | 多楼层任务规划，非替代 2D costmap |
| **开放词汇语义** | CLIP-Fields, OpenScene | 自然语言地标查询 |
| **神经隐式 SLAM** | NeRF-SLAM, GS-SLAM, SNI-SLAM | 稠密重建；规划接口未标准化 |
| **GPU 体素** | OctoMap-RT, nvblox | 大规模实时 3D 占据 |
| **地图–规划联合学习** | Neural Planner + 隐式地图 | 安全可解释性仍待验证 |

**工程侧近期优先**（Autonomy 路线图）：(1) 恢复 traversability 桥接；(2) 可选 ESDF 图层；(3) 语义 filter 接入 GridMap 层；(4) 地图版本与配置协同管理。

---

## 5.14 参考文献

**综述**

1. [Cadena et al., *Past, Present, and Future of SLAM* (2016)](https://doi.org/10.1109/TRO.2016.2624754)
2. [Garg et al., *Semantics for Robotic Mapping* (2020)](https://doi.org/10.1561/2300000051)
3. [Saarinen et al. / Annual Reviews, *Scene Representations for Robotic Spatial Perception* (2024)](https://doi.org/10.1146/annurev-control-040423-030709)
4. [Tosi et al., *NeRFs and 3D Gaussian Splatting reshaping SLAM* (2024)](https://arxiv.org/abs/2402.13255)

**里程碑论文（1995–2025 精选）**

| 年份 | 论文 | 贡献 |
|------|------|------|
| 1996 | [Thrun, *Probabilistic Robotics* mapping survey](https://robots.stanford.edu/papers/thrun.mapping-tr.pdf) | 概率占据栅格框架 |
| 2002 | [Montemerlo & Thrun, FastSLAM](https://doi.org/10.1023/A:1023783219439) | 可扩展 RBPF-SLAM |
| 2004 | [Kuipers, Hybrid SSH](https://doi.org/10.1109/ROBOT.2004.1302485) | 度量–拓扑混合 |
| 2008 | [Cummins & Newman, FAB-MAP](https://doi.org/10.1177/0278364907086282) | 外观拓扑 SLAM |
| 2010 | [Grisetti et al., g2o tutorial](https://doi.org/10.1109/MITS.2010.939365) | 图优化 SLAM |
| 2013 | [Hornung et al., OctoMap](https://doi.org/10.1007/s10514-012-9321-0) | 3D 概率八叉树 |
| 2014 | [Fankhauser et al., grid_map](https://doi.org/10.1109/IROS.2014.6942672) | 2.5D 多层栅格 |
| 2016 | [Hess et al., Cartographer](https://doi.org/10.1109/IROS.2016.7759448) | 子图 SLAM |
| 2017 | [Oleynikova et al., Voxblox](https://doi.org/10.1109/LRA.2017.2659858) | 实时 ESDF |
| 2020 | [Mildenhall et al., NeRF](https://doi.org/10.1007/978-3-030-58452-8_24) | 神经辐射场 |
| 2020 | [Rosinol et al., Kimera](https://doi.org/10.1109/ICRA.2020.9197539) | 度量–语义 SLAM |
| 2022 | [Hughes et al., Hydra](https://doi.org/10.1177/02783649241229725) | 实时 3D 场景图 |
| 2023 | [Kerbl et al., 3D Gaussian Splatting](https://doi.org/10.1145/3592433) | 显式辐射场原语 |
| 2024 | [Schmid et al., Khronos](https://www.roboticsproceedings.org/rss20/p081.pdf) | 动态时空语义 SLAM |

**工程**

- [Navigation2 Costmap2D](https://docs.nav2.org/configuration/packages/configuring-costmaps.html)
- [ETH grid_map](https://github.com/ANYbotics/grid_map)
- [OctoMap](https://octomap.github.io/)

---

## 5.15 相关文档

- [§0 指南](00_guide.md) · [§1 架构](01_architecture.md) · [§2 组件总览](02_map_components.md)
- [Costmap2D](03_costmap2d.md) · [GridMap](04_grid_map.md)
- [Planning 路径规划综述](../08_Planning/06_survey.md)
