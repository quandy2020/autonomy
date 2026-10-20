# 6. 路径规划算法综述

本章规划器输出 **无时间戳的几何 Path**；局部避障、速度、到点判定在 [09 Control](../09_Control/05_survey.md)。上手 [00 指南](00_guide.md)；插件细节 [02–05](02_planner_algorithms.md)。

## 仓库里有什么

已接入 `PlannerServer` 的 GlobalPlanner 插件：

- **NavFn**（`navfn_planner`）：势场 + 梯度跟踪，可选 A*
- **Dijkstra**（`dijkstra_planner`）：NavFn 强制 $h\equiv 0$，确定性基线
- **Theta\***（`theta_star_planner`）：栅格 A* + 视线，路径更直

可选后处理：PathSimplifier / SimpleSmoother（配置开启时）。

**没有** 接入的（勿在 conf 里指望）：Hybrid A*、RRT*、D* Lite、SMAC 等。

## 分类对照

| 族 | 代表算法 | 本仓库 |
|----|----------|--------|
| 图搜索 / 导航函数 | Dijkstra、A*、NavFn | NavFn、Dijkstra |
| 任意角 | Theta* | Theta* |
| 采样 / 曲线优化 | RRT*、CHOMP | 未接入 |

## 怎么选

| 场景 | 插件 |
|------|------|
| 默认室内网格 | `navfn_planner` |
| 大地图、要更快传播 | NavFn `use_astar=true` |
| 开阔、想要更短更直 | `theta_star_planner`（窄廊可改 4-连通） |
| 回归测试、对比基线 | `dijkstra_planner` |

决策简则：需要任意角折线 → Theta*；否则 NavFn（大图开 A*）。

## 能力边界

- 输入：起点、终点、Costmap2D **快照**（复制后搜索，不长期持锁）。
- 输出：Pose 序列；不含 $v,\omega$。
- 失败码：起点/终点在障、NO_PATH、TIMEOUT、TF_ERROR 等见 [01 架构](01_architecture.md#15-结果码)。

Costmap 不健康时换 planner 无效，先查 [07 Map](../07_Map/00_guide.md)。
