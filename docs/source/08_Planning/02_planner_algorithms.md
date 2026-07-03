# 2. 全局规划器算法

Autonomy `planning` 内置三种 `GlobalPlanner` 插件，均在全局 Costmap2D 快照上运行。本文是 **§3–§5 专题的索引与对比**；形式化见 [§0.3](00_guide.md#03-问题形式化)，架构见 [§1](01_architecture.md)，谱系与选型见 [§6 综述](06_survey.md)。

---

## 2.1 插件一览

| 插件 ID | 文档 | 策略 | 路径几何 | 状态 |
|---------|------|------|----------|------|
| `navfn_planner` | [§3 NavFn](03_navfn.md) | 导航势场 $\phi$；可选 A* | 网格 + 梯度 | ✅ |
| `dijkstra_planner` | [§4 Dijkstra](04_dijkstra.md) | 同上，$h\equiv 0$ | 网格 + 梯度 | ✅ |
| `theta_star_planner` | [§5 Theta*](05_theta_star.md) | A* + LOS | 任意角折线 | ✅ |

**继承关系**

```
GlobalPlanner
├── NavfnPlanner          ← use_astar 可选
│   └── DijkstraPlanner   ← 强制 Dijkstra
└── ThetaStarPlanner
```

NavFn 系与代码 API 的势场约定（`setGoal`/`setStart` 对调）见 [§1.3.1](01_architecture.md#131-navfn-势场约定)。

---

## 2.2 对比矩阵

| 维度 | NavFn | Dijkstra | Theta* |
|------|-------|----------|--------|
| 核心 | Eikonal 势场 + 桶队列 | NavFn 强制 Dijkstra 模式 | 栅格 A* + 视线 |
| 启发式 | 可选 A*（`use_astar`） | 无（确定性） | $h=\|n-q_g\|_2$ |
| 路径长度 | 中（锯齿） | 同 NavFn | 短（近欧氏） |
| 计算 | 快 | 快 | 中（LOS 开销） |
| 窄通道 | 较好 | 较好 | 8-连通易切角 → 改 4-连通 |
| 典型场景 | 默认室内 | 回归 / 基线 | 开阔 / office |

量级参考见 [§6.6.2](06_survey.md#662-量化参考500500-栅格量级)。

---

## 2.3 选型速查

| 场景 | 推荐 | 关键配置 |
|------|------|----------|
| 通用室内 | `navfn_planner` | 默认 |
| 确定性测试 | `dijkstra_planner` | — |
| 大地图 | `navfn` + `use_astar=true` | 提前终止 |
| 短路径 / 开阔 | `theta_star_planner` | `how_many_corners=8` |
| 窄走廊 | `navfn` 或 Theta* 4-连通 | `how_many_corners=4` |

完整矩阵与决策树见 [§6.6](06_survey.md#66-autonomy-内置规划器选型)。

---

## 2.4 公共流水线

$$
(q_s, q_g, C) \xrightarrow{\mathrm{copy}} \xrightarrow{\mathrm{search}} \xrightarrow{\mathrm{extract}} \xrightarrow{\mathrm{smooth}} \mathrm{Path}
$$

| 阶段 | NavFn / Dijkstra | Theta* |
|------|------------------|--------|
| 搜索 | 势场传播 + 梯度跟踪 | 优先队列 + LOS |
| 后处理 | `PathSimplifier` / `SimpleSmoother`（可选） | 同左 |

配置见 [§0.5](00_guide.md#05-配置与-api)；单次 `GetPlan` 时序见 [§1.3](01_architecture.md#13-单次规划流程)。

---

## 2.5 扩展阅读

| § | 文档 | 内容 |
|---|------|------|
| 3 | [NavFn](03_navfn.md) | 势场模型、传播、梯度提取 |
| 4 | [Dijkstra](04_dijkstra.md) | SSSP、Dial 桶队列、与 A* 关系 |
| 5 | [Theta*](05_theta_star.md) | LOS、任意角、邻域配置 |
| 6 | [综述](06_survey.md) | 历史、分类、鱼骨图、业界生态 |
