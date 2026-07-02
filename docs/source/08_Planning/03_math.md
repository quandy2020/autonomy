(planning-math)=
# 3. 数学原理

> 完整推导、时间轴与算法对比见 [09_survey.md](09_survey.md)；各算法实现细节见 [06_navfn.md](06_navfn.md)、[08_theta_star.md](08_theta_star.md)。

### 3.1 问题形式化

自由空间 $\mathcal{C}_{\mathrm{free}} = \mathcal{C} \setminus \mathcal{C}_{\mathrm{obs}}$，求路径 $\tau: [0,1] \to \mathcal{C}_{\mathrm{free}}$，使 $\tau(0)=q_s$，$\tau(1)=q_g$，最小化：

$$
J(\tau) = \int_0^1 \Big( w_l \|\tau'(s)\| + w_c \, c(\tau(s)) \Big) \, ds
$$

栅格离散化（`worldToMap`）：

$$
m_x = \left\lfloor \frac{x - x_0}{\Delta} \right\rfloor, \quad
m_y = \left\lfloor \frac{y - y_0}{\Delta} \right\rfloor
$$

### 3.2 代价地图

| 值 | 含义 | 规划行为 |
|----|------|----------|
| 0 | FREE | 自由通行 |
| 1–252 | 梯度代价 | 距障碍越近代价越高 |
| 253 / 254 | INSCRIBED / LETHAL | 阻塞（Theta*）/ 不可通行 |
| 255 | UNKNOWN | 由 `allow_unknown` 决定 |

NavFn 内部映射：$F_{ij} = 50 + 0.8 \cdot c_{ij}$（`COST_NEUTRAL` + `COST_FACTOR`）。

### 3.3 NavFn 导航势场

近似 Eikonal 方程 $\|\nabla\phi\| = F$，从 goal 反向传播（代码中 `setGoal(start)`、`setStart(goal)`）。

单元更新（`updateCell`）核心：

$$
\phi_n = t_a + h \cdot v\!\left(\frac{\Delta_c}{h}\right), \quad
v(r) = -0.2301 r^2 + 0.5307 r + 0.7040
$$

路径提取：沿 $-\nabla\phi$ 梯度下降，步长 $\delta=0.5$ 格。→ 详见 [06_navfn.md](06_navfn.md)

### 3.4 Dijkstra 与 A*

| 模式 | 配置 | 排序键 | 插件 |
|------|------|--------|------|
| Dijkstra | `use_astar=false` | $f=g$ | `dijkstra_planner` / NavFn |
| A* | `use_astar=true` | $f=g+h$，$h=\|n-q_s\|_2 \cdot C_n$ | NavFn |

Dijkstra 松弛：$d(v) = \min\{ d(u) + w(u,v) \}$，NavFn 用桶队列实现。→ 详见 [07_dijkstra.md](07_dijkstra.md)

### 3.5 Theta*

A* + 视线检测（LOS）：若 $\mathrm{LOS}(p, n)$ 则 $\pi(n)=p$，否则 $\pi(n)=s$。

$$
g(n) = g(p) + w_e \|p-n\|_2 + w_t \tau(n)
$$

→ 详见 [08_theta_star.md](08_theta_star.md)

### 3.6 后处理

SimpleSmoother 能量函数：

$$
E = w_d \sum \|p_i - o_i\|^2 + w_s \sum \|p_{i-1} - 2p_i + p_{i+1}\|^2
$$

### 3.7 算法一览

| 算法 | 扩展规则 | 路径几何 | Autonomy 插件 |
|------|----------|----------|---------------|
| Dijkstra | $d(v)=\min\{d(u)+w\}$ | 网格+梯度 | `dijkstra_planner` |
| A* | $f=g+h$ | 网格+梯度 | `navfn_planner` |
| Theta* | A* + LOS | 任意角直线 | `theta_star_planner` |

### 3.8 规划流水线

$$
(q_s, q_g, C)
\xrightarrow{\mathrm{discretize}}
\mathrm{costmap\ copy}
\xrightarrow{\mathrm{search}}
\mathrm{path\ extraction}
\xrightarrow{\mathrm{tolerance + smoothing}}
\mathrm{Path}
$$
