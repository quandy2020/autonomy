# 8. 地图表示综述（Survey）

本文从**学术发展史、地图表示体系、工程实践、Autonomy 定位**四个维度，系统综述移动机器人环境地图（Environmental Mapping）领域，并明确 `autonomy/map` 模块的能力边界与选型依据。

> 公式推导见 [Map 指南 · 数学原理](03_math.md)；实现细节见 [架构设计](05_architecture.md) 与各子文档。

---

## 8.1 概述：地图在导航栈中的位置

移动机器人导航通常依赖**环境地图**作为规划与感知的空间基准：

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

## 8.2 地图表示的形式化

### 8.2.1 占据栅格（Occupancy Grid）

环境 $\mathcal{W} \subset \mathbb{R}^2$ 离散为 $N_x \times N_y$ 栅格，每格存储占据概率或标签：

$$
p(m_{ij} = \mathrm{occupied} \mid z_{1:t})
$$

典型输出：$o_{ij} \in \{-1, 0, \ldots, 100\}$（未知 / 自由 / 占据百分比）。

**Autonomy 实现**：`MapServer` 加载 YAML+PGM，`StaticLayer` 消费 `OccupancyGrid`。

### 8.2.2 代价地图（Costmap）

在占据栅格基础上引入**通行代价** $c_{ij} \in [0, 255]$，使规划器不仅避开障碍，还能**远离障碍**：

$$
c_{ij} = f(d_{ij}^{\text{obs}}, r_i, \lambda)
$$

其中 $d_{ij}^{\text{obs}}$ 为到最近障碍的距离，$r_i$ 为机器人内切半径，$\lambda$ 为衰减系数。

**Autonomy 实现**：`Costmap2D` + `InflationLayer`，指数衰减公式见 [§3.6](03_math.md#36-膨胀层inflationlayer数学)。

### 8.2.3 2.5D 多层栅格（Grid Map）

在 XY 平面维护多层浮点属性 $\{L_k\}$，第三维信息编码在层值中：

$$
\mathcal{M}_{2.5D} = \{ L_k(i,j) \mid k = 1,\ldots,K \}
$$

典型层：高程 $z$、方差 $\sigma_z^2$、坡度、法向量、可通行性。

**Autonomy 实现**：`grid_map::GridMap`，ETH `grid_map` 架构。

### 8.2.4 其他表示（Autonomy 未内置）

| 表示 | 特点 | 典型应用 |
|------|------|----------|
| 点云地图 | 精确 3D，数据量大 | LOAM、LIO-SAM |
| 八叉树 (OctoMap) | 稀疏 3D 占据 | 无人机、机械臂 |
| 拓扑地图 | 节点+边，语义丰富 | 多楼层导航 |
| 符号地图 | 物体级语义 | 人机交互 |
| ESDF / TSDF | 有符号距离场 | 实时避障、Voxblox |

---

## 8.3 发展时间轴

<div class="planning-timeline-v2">

<div class="timeline-era-block era-foundation">
  <div class="timeline-era-header">奠基期 · 1980s–1990s</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1985</div>
      <div class="timeline-milestone-title">Occupancy Grid</div>
      <div class="timeline-milestone-desc">Moravec & Elfes，概率占据栅格</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1986</div>
      <div class="timeline-milestone-title">Configuration Space</div>
      <div class="timeline-milestone-desc">Lozano-Pérez，构型空间障碍映射</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1990s</div>
      <div class="timeline-milestone-title">SLAM</div>
      <div class="timeline-milestone-desc">同时定位与建图，地图在线更新</div>
    </div>
  </div>
</div>

<div class="timeline-era-block era-reactive">
  <div class="timeline-era-header">ROS 时代 · 2000s–2010s</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2008</div>
      <div class="timeline-milestone-title">costmap_2d</div>
      <div class="timeline-milestone-desc">Willow Garage，多层代价地图</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2013</div>
      <div class="timeline-milestone-title">grid_map</div>
      <div class="timeline-milestone-desc">ETH，2.5D 多层浮点栅格</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2014</div>
      <div class="timeline-milestone-title">OctoMap</div>
      <div class="timeline-milestone-desc">3D 概率八叉树地图</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2018</div>
      <div class="timeline-milestone-title">Nav2</div>
      <div class="timeline-milestone-desc">ROS 2 导航栈，costmap 插件化</div>
    </div>
  </div>
</div>

<div class="timeline-era-block era-modern">
  <div class="timeline-era-header">现代 · 2020s</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2020s</div>
      <div class="timeline-milestone-title">语义地图</div>
      <div class="timeline-milestone-desc">物体级、场景图、LLM 导航</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2024</div>
      <div class="timeline-milestone-title">Autonomy map</div>
      <div class="timeline-milestone-desc">Nav2 costmap + ETH grid_map 融合</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2020s</div>
      <div class="timeline-milestone-title">NeRF / 3DGS</div>
      <div class="timeline-milestone-desc">神经辐射场，隐式场景表示</div>
    </div>
  </div>
</div>

</div>

---

## 8.4 地图表示对比

| 维度 | OccupancyGrid | Costmap2D | GridMap 2.5D | OctoMap |
|------|---------------|-----------|--------------|---------|
| 维度 | 2D | 2D | 2.5D | 3D |
| 数值类型 | int8 (-1/0/100) | uint8 (0–255) | float 多层 | float 概率 |
| 更新方式 | 静态 / SLAM | 多层插件融合 | 循环 buffer | 概率更新 |
| 规划用途 | 静态障碍 | 全局/局部规划 | 地形/坡度 | 3D 避障 |
| 内存 | $O(N_x N_y)$ | $O(N_x N_y)$ | $O(K \cdot N_x N_y)$ | 稀疏 |
| Autonomy | ✅ MapServer | ✅ Costmap2D | ✅ GridMap | ❌ |

---

## 8.5 Costmap2D 图层体系

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

详见 [Costmap2D 文档](06_costmap2d.md#69-图层详解)。

---

## 8.6 膨胀算法的物理意义

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

## 8.7 GridMap 2.5D 的设计哲学

ETH `grid_map` 的设计目标：

1. **多层解耦**：不同传感器/算法写入不同层，互不干扰
2. **循环 buffer**：$O(1)$ 滑动窗口，无需全图 copy
3. **连续插值**：支持 sub-cell 精度查询
4. **Eigen 集成**：矩阵运算、滤波器无缝对接

**2.5D vs 3D**：不维护完整 3D 体素，而是用层值编码高度信息，在计算效率与表达能力间折中。

---

## 8.8 Autonomy map 模块定位

| 能力 | 状态 | 说明 |
|------|------|------|
| 静态地图服务 | ✅ 已实现 | `MapServer`，YAML+PGM |
| 2D 代价地图 | ✅ 已实现 | Nav2 兼容，插件化图层 |
| 2.5D GridMap | ✅ 已实现 | ETH grid_map 封装 |
| Costmap ↔ GridMap 桥接 | ❌ 未实现 | `grid_map_costmap_2d/` 已注释 |
| 3D 体素地图 | 部分 | `VoxelLayer` 投影到 2D |
| 语义地图 | ❌ | 需外部模块 |

**推荐选型**：

| 场景 | 推荐组件 |
|------|----------|
| 室内平面导航 | Costmap2D（static + obstacle + inflation） |
| 已知静态环境 | MapServer + static_layer |
| 动态障碍环境 | + obstacle_layer + 激光 |
| 越野 / 台阶 | GridMap（elevation + slope） |
| 窄通道 / 精确定位 | 减小 inflation_radius，禁用 denoise |

---

## 8.9 与 Planning 的耦合

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

## 8.10 影响因素与调参

| 因素 | 影响 | 调参方向 |
|------|------|----------|
| 分辨率 $\Delta$ | 精度 vs 内存/速度 | 室内 0.05 m，室外 0.1–0.2 m |
| 膨胀半径 $R$ | 安全裕度 vs 窄通道 | 0.35–0.55 m |
| 更新频率 | 动态响应 vs CPU | 5 Hz 通常足够 |
| Rolling window | 大地图内存 | 局部规划启用 |
| denoise | 噪声过滤 vs 薄墙保留 | 谨慎启用 |
| footprint | 碰撞模型精度 | 多边形优于圆形 |

---

## 8.11 未来方向

1. **恢复 `grid_map_costmap_2d` 桥接**：elevation / traversability → costmap 代价
2. **ESDF 图层**：更精确的距离场，支持梯度优化规划
3. **语义层融合**：GridMap 语义层 → 规划代价加权
4. **动态地图版本管理**：配置与地图文件协同版本化
5. **GPU 加速**：大规模 costmap 更新与 GridMap 插值

---

## 8.12 相关文档

- [Map 地图模块指南](00_guide.md)
- [数学原理](03_math.md)
- [Costmap2D 代价地图](06_costmap2d.md)
- [GridMap 2.5D 栅格地图](07_grid_map.md)
- [模块架构设计](05_architecture.md)
- [Planning 路径规划综述](../08_Planning/06_survey.md)
