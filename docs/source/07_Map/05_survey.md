# 5. 地图表示综述

> 实现 [§3](03_costmap2d.md) · [§4](04_grid_map.md)；上手 [§0](00_guide.md)。

### 5.1 在导航栈中

| 子问题 | 表示 | Autonomy |
|--------|------|----------|
| 静态图 | OccupancyGrid | MapServer |
| 规划/避障 | Costmap | Costmap2D |
| 地形 | 2.5D 多层 | GridMap |

```
SLAM/文件 → MapServer → static → obstacle → inflation → Planning
深度/点云 → GridMap（并行）
```

### 5.2 形式化

占据：$p(m_{ij}=\mathrm{occ}\mid z_{1:t})$，$o\in\{-1,0..100\}$。

代价：$c_{ij}=f(d_{ij}^{\mathrm{obs}},r_i,\lambda)\in[0,255]$。

2.5D：$\mathcal{M}=\{L_k(i,j)\}$（elevation / variance / slope …）。

| 未内置 | 用途 |
|--------|------|
| OctoMap / ESDF | 3D 占据 / 距离场 |
| 拓扑 / 语义图 | 多楼层、任务 |
| 点云稠密图 | LIO 建图侧 |

### 5.3 分类对照

| 族 | 代表 | 规划直接用 | Autonomy |
|----|------|------------|----------|
| 2D 占据 | OccupancyGrid | 经 static | — |
| 2D 代价 | Costmap | — | |
| 2.5D | GridMap | 需桥接 | — |
| 3D 体素 | OctoMap | 投影后 | — |

### 5.4 选型

| 场景 | 选 |
|------|-----|
| 室内 2D 导航 | Costmap 三层标准配置 |
| 仅加载地图 | MapServer |
| 越野 / 坡度 | GridMap + 应用映射 |
| 大范围局部 | rolling Costmap |
