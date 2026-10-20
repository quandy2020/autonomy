# 2. 全局规划器算法

三种插件均在 Costmap2D 快照上运行。形式化 [§0.3](00_guide.md)；选型 [§6](06_survey.md)。

### 2.1 一览

| ID | 策略 | 几何 | 文档 |
|----|------|------|------|
| `navfn_planner` | 势场 φ，可选 A* | 网格+梯度 | [§3](03_navfn.md) |
| `dijkstra_planner` | NavFn，$h\equiv0$ | 同上 | [§4](04_dijkstra.md) |
| `theta_star_planner` | A* + LOS | 任意角折线 | [§5](05_theta_star.md) |

```
GlobalPlanner → NavfnPlanner → DijkstraPlanner
→ ThetaStarPlanner
```

### 2.2 对比

| — | NavFn | Dijkstra | Theta* |
|--|-------|----------|--------|
| 启发式 | 可选 | 无 | $\|n-q_g\|_2$ |
| 路径 | 中（锯齿） | 同左 | 短 |
| 算力 | 快 | 快 | 中（LOS） |
| 场景 | 默认室内 | 基线 | 开阔 |

### 2.3 选型

| 场景 | 推荐 |
|------|------|
| 通用室内 | `navfn_planner` |
| 大地图 | `navfn` + `use_astar` |
| 短路径 | `theta_star`（窄廊可用 4-连通） |

流水线：复制 → 搜索 → 提取 → 可选平滑。
