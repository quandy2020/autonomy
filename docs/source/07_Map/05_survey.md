# 5. 地图表示综述

导航栈里「地图」分三条线：静态占据、规划代价、2.5D 地形。实现细节见 [03 Costmap2D](03_costmap2d.md)、[04 GridMap](04_grid_map.md)；上手 [00 指南](00_guide.md)。

## 在栈中的位置

```text
SLAM / YAML+PGM → MapServer → OccupancyGrid
                                    ↓
                              static_layer ─┐
/scan ──────────────────► obstacle_layer ──┼─► Costmap2D → Planning / Control
                              inflation_layer ┘

点云 / Follow ──► GridMap（并行，不自动进 Costmap）
```

| 子问题 | 表示 | Autonomy 组件 |
|--------|------|----------------|
| 静态图可视化、static_layer | OccupancyGrid | MapServer |
| 路径搜索、动态障 | uint8 Costmap | Costmap2DWrapper |
| 坡度、跟人 | float 多层 | GridMapWrapper |

## 形式化（查阅用）

占据：单元 $m_{ij}$ 占用概率；ROS 消息常用 $o\in\{-1,0..100\}$。

代价：$c_{ij}=f(d^{\mathrm{obs}}, r_i, \lambda)\in[0,255]$，254 致命。

2.5D：$\mathcal{M}=\{L_k(i,j)\}$，层名如 elevation / variance / slope。

## 仓库里尚未作为一等公民的

OctoMap、ESDF、拓扑/语义图、稠密点云地图等多在 **定位侧**（Atlas/Argus 点云图）或外部工具；**全局 planner 插件** 当前只读 2D Costmap 快照。

## 怎么选

| 场景 | 选 |
|------|-----|
| 室内 2D 导航 | Costmap：static + obstacle + inflation |
| 只加载 PGM 给 RViz | MapServer |
| 越野 / 坡度约束 | GridMap + 自写映射或 MPPI critic |
| 大范围户外局部窗 | Costmap `rolling_window=true` |

Costmap 与 GridMap 原点不同，联调时先画同一世界点对齐，再谈融合策略。
