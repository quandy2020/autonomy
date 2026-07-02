# 8. Theta* 全局规划器

`ThetaStarPlanner` 实现 **Theta*** 任意角度网格路径规划，源自 nav2 `nav2_theta_star_planner`。实现位于 `autonomy/planning/planner/theta_star/`。

> 完整数学推导见 [Planning 指南 · 数学公式](03_math.md) §3.5。

## 8.1 算法概述

### 8.1.1 网格路径的次优性

8-连通网格路径长度满足：

$$
L_{\mathrm{grid}} \geq \|q_a - q_b\|_2
$$

等号仅当路径与坐标轴对齐。Theta* 通过**视线捷径**使路径逼近欧氏直线：

$$
L_{\mathrm{Theta*}} \approx \sum_k \|q_{k+1} - q_k\|_2 \approx \|q_g - q_s\|_2
$$

### 8.1.2 算法分类

| 属性 | 值 |
|------|-----|
| 搜索空间 | 4/8 连通栅格 |
| 最优性 | Any-angle 近似最优 |
| 路径类型 | 任意角度直线段 |
| 核心 | A* + Line-of-Sight |

## 8.2 数学原理

### 8.2.1 评估函数

标准 A* 框架，按 $f(n) = g(n) + h(n)$ 扩展开放列表：

$$
g(q_s) = 0, \quad \pi(q_s) = q_s
$$

### 8.2.2 Theta* 更新规则

对当前格 $s$、父节点 $p = \pi(s)$、邻居 $n$：

**若 $\mathrm{LOS}(p, n) = \mathrm{true}$**（视线畅通）：

$$
g(n) = g(p) + w_e \| p - n \|_2 + w_t \cdot \tau(n)
$$

$$
\pi(n) = p
$$

**否则**（标准 A*）：

$$
g(n) = g(s) + w_e \| s - n \|_2 + w_t \cdot \tau(n)
$$

$$
\pi(n) = s
$$

其中通行代价：

$$
\tau(n) = w_t \cdot \frac{c_n}{252}
$$

### 8.2.3 启发式

$$
h(n) = w_h \cdot \| n - q_g \|_2
$$

| 参数 | 默认 | 含义 |
|------|------|------|
| $w_e$ (`w_euc_cost`) | 2.0 | 欧氏距离权重 |
| $w_t$ (`w_traversal_cost`) | 1.0 | 代价地图权重 |
| $w_h$ (`w_heuristic_cost`) | 1.0 | 启发式权重 |

当 $w_e < 1.0$ 时自动令 $w_h = w_e$，保证 $h$ 可采纳。

### 8.2.4 视线检测

$\mathrm{LOS}(a, b)$ 用 Bresenham 算法遍历线段上所有栅格。任一格阻塞则返回 false：

$$
\mathrm{blocked}(i,j) \Leftrightarrow
c_{ij} \in \{253, 254\} \lor (c_{ij}=255 \land \neg u_{\mathrm{unk}})
$$

其中 $u_{\mathrm{unk}}$ 为 `allow_unknown` 标志。

### 8.2.5 路径重建与朝向

从 $q_g$ 回溯 $\pi$ 链，反转后得路径。中间点朝向：

$$
\theta_i = \operatorname{atan2}(y_{i+1}-y_i,\; x_{i+1}-x_i)
$$

末端使用 goal 原始位姿 $(x_g, y_g, \theta_g)$。

## 8.3 源码对照

Theta* 核心更新（`theta_star_planner.cpp`）：

```cpp
if (lineOfSight(px, py, unx, uny)) {
    tentative_g = g_score[par_idx]
        + w_euc_cost_ * euclideanDistance(px, py, unx, uny)
        + traversalCost(unx, uny);
    new_parent = par_idx;
} else {
    tentative_g = g_score[curr_idx]
        + w_euc_cost_ * euclideanDistance(cx, cy, unx, uny)
        + traversalCost(unx, uny);
    new_parent = curr_idx;
}
```

## 8.4 邻域扩展

| `how_many_corners` | 邻居 | 适用 |
|--------------------|------|------|
| 4 | 上下左右 | 窄通道，防切角 |
| 8 | + 四对角 | 默认，更灵活 |

## 8.5 配置参数

```lua
theta_star_planner = {
    how_many_corners = 8,
    allow_unknown = false,
    w_euc_cost = 2.0,
    w_traversal_cost = 1.0,
    w_heuristic_cost = 1.0,
    terminal_checking_interval = 5000,
},
```

## 8.6 参数调优

| 目标 | 调整 |
|------|------|
| 更短路径 | 增大 $w_e$，减小 $w_t$ |
| 更安全 | 增大 $w_t$ |
| 窄通道 | `how_many_corners = 4` |
| 更快搜索 | 适当减小 $w_h$（保持可采纳） |

## 8.7 复杂度

设 $N$ 为栅格数，$\bar{L}$ 为平均 LOS 检测长度：

| 阶段 | 复杂度 |
|------|--------|
| 复制 costmap | $O(N)$ |
| A* + LOS | $O(N \log N \cdot \bar{L})$ |
| 回溯 | $O(P)$，$P$ 为路径点数 |

## 8.8 与其他规划器对比

| 维度 | Theta* | NavFn/Dijkstra |
|------|--------|----------------|
| 路径角度 | 任意 | 网格 |
| 路径长度 | $\approx \|q_g-q_s\|$ | $\geq \|q_g-q_s\|$ |
| 计算量 | 较高（LOS） | 较低 |
| 窄通道 | 4-连通更安全 | 稳定 |

## 8.9 参考文献

- [Nash et al., "Theta*: Any-Angle Path Planning on Grids", AAAI 2007](https://cdn.aaai.org/AAAI/2007/AAAI07-187.pdf)
- [nav2_theta_star_planner](https://github.com/ros-navigation/navigation2/tree/main/nav2_theta_star_planner)
- [Planning 指南 · 数学公式](03_math.md)
