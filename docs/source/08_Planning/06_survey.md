# 6. 路径规划算法综述

> 上手 [§0](00_guide.md)；插件 [§2–§5](02_planner_algorithms.md)。

### 6.1 能力边界

全局栅格几何路径；局部避障/速度在 `control`。

| 能力 | 状态 |
|------|------|
| NavFn / Dijkstra / A* / Theta* | — |
| 简化 / 平滑后处理 | — |
| Hybrid A* / RRT* / D* Lite | — |

### 6.2 分类

| 族 | 代表 | Autonomy |
|----|------|----------|
| 图搜索 | Dijkstra, A* | — |
| 导航函数 | NavFn | — |
| 任意角 | Theta* | — |
| 采样 / 优化 / 混合 | RRT*, CHOMP, Hybrid A* | — |

### 6.3 选型

| 场景 | 推荐 |
|------|------|
| 默认室内 | `navfn_planner` |
| 大图加速 | `use_astar=true` |
| 开阔短路径 | `theta_star_planner` |
| 确定性回归 | `dijkstra_planner` |

决策：要任意角？→ Theta*；否则 NavFn（大图开 A*）。

### 6.4 与 Control

Planning 出无时间 Path；局部轨迹 / cmd_vel 见 [Control 综述](../09_Control/05_survey.md)。
