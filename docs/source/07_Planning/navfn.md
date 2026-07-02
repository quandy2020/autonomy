# NavFn 全局规划器

NavFn（Navigation Function）是 Autonomy 默认的全局路径规划器，源自 ROS 1 `navfn` 包。实现位于 `autonomy/planning/planner/navfn/`。

> 完整数学推导见 [Planning 指南 · 数学公式](guide.md#planning-math)。

## 1. 算法概述

NavFn 求解离散栅格上的**导航势场** $\phi(i,j)$：从每个自由格到目标的最小通行代价。路径通过对势场做**梯度下降跟踪**得到。

核心两步：

1. **反向传播**：从目标格 $q_g$ 出发，计算 $\phi(q)$，满足近似 Eikonal 方程 $\|\nabla\phi\| = F$
2. **梯度跟踪**：从 $q_g$ 沿 $-\nabla\phi$ 走到 $q_s$

> **注意**：代码中 `setGoal()` 传入用户 start、`setStart()` 传入用户 goal，因为传播方向与提取方向相反。详见数学文档 §3.2。

### 1.1 算法分类

| 属性 | 值 |
|------|-----|
| 搜索空间 | 二维栅格（8-连通隐式） |
| 最优性 | 离散网格上近似最优 |
| 完备性 | 自由空间连通时完备 |
| 路径类型 | 网格 + 子像素梯度插值 |
| 时间复杂度 | $O(N)$（Dijkstra 桶队列）或 $O(N \log N)$（A*） |

## 2. 数学原理

### 2.1 Eikonal 方程与平面波近似

连续模型中，势场 $\phi$ 满足：

$$
\|\nabla \phi(x,y)\| = F(x,y), \quad \phi(q_g) = 0
$$

NavFn 不直接解 Eikonal，而是用**平面波更新**在 4-邻域上近似。对格 $n$，令 $t_a = \min(\phi_u, \phi_d)$，$t_c = \min(\phi_l, \phi_r)$，$h = F_n$：

$$
\phi_n = \begin{cases}
t_a + h & \Delta_c \geq h \\[6pt]
t_a + h \cdot v\!\left(\dfrac{\Delta_c}{h}\right) & \text{otherwise}
\end{cases}
$$

其中 $\Delta_c = |t_c - t_a|$，插值函数：

$$
v(r) = -0.2301\,r^2 + 0.5307\,r + 0.7040
$$

对应 `NavFn::updateCell()`（`navfn.cpp`）。

### 2.2 代价映射

Costmap 值 $c \in [0,252]$ 映射为 NavFn 通行代价：

$$
F_{ij} = 50 + 0.8 \cdot c_{ij}
$$

| 常量 | 值 | 含义 |
|------|-----|------|
| `COST_NEUTRAL` | 50 | 开放空间基准 |
| `COST_FACTOR` | 0.8 | 线性缩放 |
| `COST_OBS` | 254 | 不可通行 |
| `POT_HIGH` | $10^{10}$ | 未访问标记 |

### 2.3 梯度跟踪

势场建立后，从 goal 格出发，步长 $\delta = 0.5$ 格：

$$
\mathbf{p}_{k+1} = \mathbf{p}_k - \delta \cdot \frac{\nabla\phi(\mathbf{p}_k)}{\|\nabla\phi(\mathbf{p}_k)\|}
$$

梯度用中心差分（`gradCell`）：

$$
\frac{\partial\phi}{\partial x} \approx \frac{\phi_{i-1,j}-\phi_{i+1,j}}{2}, \quad
\frac{\partial\phi}{\partial y} \approx \frac{\phi_{i,j-1}-\phi_{i,j+1}}{2}
$$

终止：$\phi(\mathbf{p}_k) < 50$ 或超过 `max_cycles = 4 \cdot \max(M,N)`。

## 3. 两种传播模式

### 3.1 Dijkstra 模式（`use_astar = false`）

按势场值递增的**桶队列**传播，排序键 $f = g = \phi_n$：

```cpp
planner_->calcNavFnDijkstra(cancel_checker, true);
//                                          atStart=true: φ(q_s) 有限即停
```

- 桶阈值步长 $\Delta T = 2 \times 50 = 100$
- `atStart=true` 时不必遍历全图

### 3.2 A* 模式（`use_astar = true`）

加入欧氏距离启发式，排序键 $f = g + h$：

$$
h(n) = \|n - q_s\|_2 \cdot C_n
$$

```cpp
planner_->calcNavFnAstar(cancel_checker);
```

启发式可采纳（$h$ 为欧氏下界），保证离散最优；大地图上通常比 Dijkstra 更快终止。

## 4. NavfnPlanner 插件流程

```
1. worldToMap(q_s), worldToMap(q_g)
2. 复制 costmap，令 c(q_s) = 0
3. setGoal(q_s), setStart(q_g)     ← 反向设置
4. calcNavFnDijkstra / calcNavFnAstar
5. 容差搜索: q* = argmin ||q - q_g||, s.t. φ(q) < POT_HIGH
6. calcPath() 梯度跟踪 → 世界坐标
7. smoothApproachToGoal() 末端修正
```

### 4.1 目标容差

当 $\phi(q_g) \geq \texttt{POT\_HIGH}$ 时，在 $\|q - q_g\|_\infty \leq \varepsilon$ 内搜索最近可达点：

$$
q^* = \arg\min_{\|q-q_g\|_\infty \leq \varepsilon} \|q - q_g\|_2
\quad \text{s.t.} \quad \phi(q) < \texttt{POT\_HIGH}
$$

默认 $\varepsilon = 0.1\,\text{m}$。

### 4.2 末端朝向

`use_final_approach_orientation=true` 时，末端朝向：

$$
\theta_N = \operatorname{atan2}(y_N - y_{N-1},\; x_N - x_{N-1})
$$

## 5. 配置参数

```lua
navfn_planner = {
    tolerance = 0.1,
    use_astar = false,
    allow_unknown = false,
    use_final_approach_orientation = false,
},
```

| 字段 | 说明 |
|------|------|
| `tolerance` | 目标容差 $\varepsilon$（米） |
| `use_astar` | `true` → A*， `false` → Dijkstra |
| `allow_unknown` | 是否穿越 UNKNOWN 格 |
| `use_final_approach_orientation` | 末端接近朝向 |

## 6. 与其他规划器的关系

```
NavfnPlanner (use_astar=false)  ≡  Dijkstra 模式
NavfnPlanner (use_astar=true)   →  A* 模式
DijkstraPlanner                 →  强制 use_astar=false
```

## 7. 调试建议

| 现象 | 可能原因 | 排查 |
|------|----------|------|
| 无路径 | $\phi(q_g) = \infty$ | 增大 `tolerance`，检查膨胀层 |
| 路径穿墙 | costmap 未更新 | 确认 obstacle/inflation 层 |
| 锯齿严重 | 网格梯度跟踪 | 换 Theta* 或启用平滑 |
| 规划慢 | 全图 Dijkstra | `use_astar = true` |

## 8. 参考文献

- Eriksson & Borenstein, "GURVEY", IEEE AES Magazine, 1990
- [nav2_navfn_planner](https://github.com/ros-navigation/navigation2/tree/main/nav2_navfn_planner)
- [Planning 指南 · 数学公式](guide.md#planning-math)
