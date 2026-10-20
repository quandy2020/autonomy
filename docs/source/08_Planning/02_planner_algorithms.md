# 2. 全局规划器算法

三种插件都在 **Costmap2D 快照** 上搜索，输出 `Path`。形式化与 NavFn 起终点对调约定见各专题页；对照 [06 综述](06_survey.md)。

按「问题 → 算法族 → 插件 id → conf 字段」阅读；默认项以代码注册为准。

| 插件 ID | 做法 | 路径形态 | 专题 |
|---------|------|----------|------|
| `navfn_planner` | 势场，可选 A* | 网格 + 梯度跟踪 | [03](03_navfn.md) |
| `dijkstra_planner` | NavFn，$h\equiv 0$ | 同上 | [04](04_dijkstra.md) |
| `theta_star_planner` | A* + LOS | 任意角折线 | [05](05_theta_star.md) |

继承：`GlobalPlanner` → `NavfnPlanner` → `DijkstraPlanner`；Theta* 独立实现。

## 对比（选型）

| | NavFn | Dijkstra | Theta* |
|--|-------|----------|--------|
| 启发式 | 可选 A* | 无 | 到目标欧氏 |
| 路径形状 | 易锯齿 | 类似 NavFn | 通常更短更直 |
| 算力 | 低 | 低 | 中（LOS 检查） |
| 终点在障附近 | 有容差搜索 | 同 NavFn | **无** 容差，$q_g$ 须可通行 |

## 怎么选

- 默认室内：`navfn_planner`。
- 地图大、传播慢：NavFn 开 `use_astar=true`。
- 开阔区域、想要更直路径：`theta_star_planner`；窄走廊改 4-连通防切角。
- CI / 回归基线：`dijkstra_planner`。

流水线共性：复制 costmap → 搜索 → 提取 pose 序列 → 可选 PathSimplifier / SimpleSmoother。

## 常见坑

| 现象 | 先查 |
|------|------|
| 三种都 NO_PATH | 起点/终点在 LETHAL；static 未加载 |
| Theta* 失败 NavFn 成功 | 目标格不可通行或 unknown 策略更严 |
| 换 id 无效 | `default_planner_id` 与 `planner_plugins` 字符串 |
