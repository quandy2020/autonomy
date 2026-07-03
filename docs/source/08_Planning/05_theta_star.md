(theta-star-planner)=
# 5. Theta* 全局规划器

> 归属 [§6.5.3 任意角规划](06_survey.md#653-theta-任意角规划) · [§0.4 代价地图](00_guide.md#04-代价地图) · Autonomy ✅ `theta_star_planner`
>
> **Theta\***（Nash et al., AAAI 2007）在栅格 A\* 扩展时加入 **Line-of-Sight（LOS）** 检测：若祖父节点 $p=\pi(s)$ 与邻居 $n$ 视线畅通，则 $\pi(n)\leftarrow p$ 并沿直线更新 $g(n)$，路径逼近欧氏最短折线。Autonomy 实现于 `autonomy/planning/planner/theta_star/`，源自 nav2 `nav2_theta_star_planner`。

---

## 1. 背景

8-连通网格 A\* 的路径长度满足 $L_{\mathrm{grid}}\ge\|q_g-q_s\|_2$，等号仅当路径与坐标轴对齐——**网格锯齿**使全局路径偏长。任意角（any-angle）规划在栅格上允许直线段连接，Theta* 在 A\* 框架内对每步扩展做即时 LOS 验证，无需后处理拉直即可得到接近 $\|q_g-q_s\|_2$ 的折线路径。Field D*（Ferguson & Stentz, 2005）等同属任意角谱系；Autonomy 选用 Theta* 作为 nav2 对齐的轻量实现（详见 §3–§5）。

---

## 2. 问题

**任务.** 给定代价地图 $C$ 与 $q_s,\, q_g \in \mathcal{C}_{\mathrm{free}}$，求无碰撞折线路径 $\tau$，使 $\tau(0)=q_s$，$\tau(1)=q_g$，并权衡欧氏长度与代价地图通行代价。

**输入 / 输出.** `ThetaStarPlanner::CreatePlan` 接收起终点 `PoseStamped`、Costmap2D 快照与 `cancel` 回调；输出 `planning_msgs::Path`（世界坐标 + 航向）。**无** NavFn 式目标容差：$q_g$ 格须可直接通行，否则 `FAIL`。

**在线形式.** 每格扩展含 LOS（Bresenham）检测，复杂度高于 NavFn；适合静态环境、开阔场景及需短路径的 office 类任务（[§6.6.3 场景选型矩阵](06_survey.md#663-场景选型矩阵)）。窄通道可设 `how_many_corners=4` 抑制对角切角（§4.2）。

---

## 3. Theta* 搜索模型

以下给出 §4 搜索与 §5 路径重建所需符号：A\* 框架 → LOS 更新 → 代价与启发式。

### 3.1 A* 评估函数

在栅格索引 $n$ 上维护累积代价 $g(n)$、父指针 $\pi(n)$ 与开放列表排序键：

$$
f(n) = g(n) + h(n), \qquad g(q_s)=0,\;\pi(q_s)=q_s.
$$

- **$g(n)$**：从 $q_s$ 到 $n$ 的加权路径代价（§3.3）。
- **$h(n)$**：到 $q_g$ 的启发式（§3.4）；$w_h\le w_e$ 时**可采纳**。

### 3.2 Theta* 更新（AAAI 2007）

扩展当前格 $s$、父节点 $p=\pi(s)$、邻居 $n\in\mathcal{N}(s)$（4/8 连通，§4.2）：

**若 $\mathrm{LOS}(p,n)=\mathrm{true}$**：

$$
g(n) \leftarrow g(p) + w_e\,\|p-n\|_2 + \tau(n), \qquad \pi(n) \leftarrow p.
$$

**否则**（标准 A* 步进）：

$$
g(n) \leftarrow g(s) + w_e\,\|s-n\|_2 + \tau(n), \qquad \pi(n) \leftarrow s.
$$

- **含义**：LOS 成立时「跳过」$s$，$p$ 直连 $n$，形成任意角直线段；否则沿网格边前进。
- **实现**：仅当 $g(n)$ 下降时更新并入队（`theta_star_planner.cpp`）。

### 3.3 通行代价

格 $n$ 的代价值 $c_n\in[0,252]$（Costmap2D 语义见 [§0.4 代价地图](00_guide.md#04-代价地图)）：

$$
\tau(n) = w_t \cdot \frac{c_n}{252}.
$$

UNKNOWN（255）且 `allow_unknown=true` 时取 $\tau(n)=w_t$；`allow_unknown=false` 时 UNKNOWN 格在 LOS / 扩展中视为阻塞。

| 参数 | 默认 | 含义 |
|------|------|------|
| $w_e$ | `2.0` | 欧氏段长权重 |
| $w_t$ | `1.0` | 代价地图权重 |
| $w_h$ | `1.0` | 启发式权重（$w_e<1$ 时自动 $w_h\leftarrow w_e$） |

### 3.4 启发式

$$
h(n) = w_h \cdot \|n - q_g\|_2.
$$

- **含义**：欧氏距离下界；$w_h\le w_e$ 时不高估 $g$，保持 A\* 可采纳性。

### 3.5 视线检测 $\mathrm{LOS}(a,b)$

Bresenham 遍历线段 $ab$ 上各格；任一格阻塞则 $\mathrm{LOS}=\mathrm{false}$：

$$
\mathrm{blocked}(i,j) \Leftrightarrow
c_{ij}\in\{253,254\} \lor (c_{ij}=255 \land \neg u_{\mathrm{unk}}).
$$

- **含义**：253/254 为 INSCRIBED / LETHAL；$u_{\mathrm{unk}}$ 为 `allow_unknown`；Theta* 不对膨胀区做 NavFn 式软代价，直接硬阻塞。

---

## 4. 搜索与路径重建

### 4.1 主搜索循环

`makePlan` 复制 costmap 并置 $c'(q_s)\leftarrow\text{FREE}$ 后，以二叉堆维护开放列表，按 $f$ 最小弹出；已关闭格跳过（lazy deletion）。`terminal_checking_interval`（默认 5000 步）调用 `cancel` 可中断搜索。

- **终止**：弹出 $q_g$ 或开放列表空（`FAIL`）。
- **复杂度**：最坏 $O(N\log N\cdot\bar{L})$，$N=n_x n_y$，$\bar{L}$ 为平均 LOS 链长（Bresenham）。

### 4.2 邻域与切角

| `how_many_corners` | $\mathcal{N}(s)$ | 适用 |
|--------------------|------------------|------|
| 4 | 4-连通 | 窄通道，减少对角切角 |
| 8 | 8-连通（默认） | 开阔场景，更短路径 |

### 4.3 路径重建与朝向

自 $q_g$ 沿 $\pi$ 回溯至 $q_s$，反转得格点序列；`mapToWorld` 后中间点航向：

$$
\theta_i = \operatorname{atan2}(y_{i+1}-y_i,\; x_{i+1}-x_i).
$$

末端航点强制为 $q_g$ 的位姿 $(x_g,y_g,\theta_g)$（`goal.pose`）。

---

## 5. 求解

§4 给出单步语义；以下 **算法 1–3** 描述 `ThetaStarPlanner::CreatePlan`（`theta_star_planner.cpp`）。

### 5.1 算法（数学描述）

<div class="algorithm-box-diagram">

<div class="algorithm-box algorithm-box-phase-a">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 1</span>
    <span class="algorithm-box-title">ThetaStarPlanner 全局规划</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{CreatePlan}(q_s,\, q_g,\, C;\, \Theta,\, \texttt{cancel}) \mapsto \mathrm{Path}\ \|\ \text{FAIL}$

  </div>
  <div class="algorithm-box-io" markdown="1">

| 方向 | 符号 | 说明 |
|------|------|------|
| 输入 | $q_s,\, q_g$ | 起点 / 终点 `PoseStamped` |
| 输入 | $C$ | 全局 Costmap2D（加锁复制快照） |
| 输入 | $\Theta$ | `how_many_corners`, `allow_unknown`, $w_e,w_t,w_h$, … |
| 输入 | `cancel` | 搜索取消回调（Alg. 2） |
| 输出 | $\mathrm{Path}$ | 世界坐标 + 航向（$q_s \to q_g$） |
| 输出 | 状态码 | `SUCCESS` / `NO_PATH` |

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&\text{worldToMap}(q_s),\;\text{worldToMap}(q_g);\;
\text{若越界则 return FAIL} \\[6pt]
\textbf{2.}\;&\text{加锁复制 } C \to C';\;
c'(q_s)\leftarrow \text{FREE} \\[6pt]
\textbf{3.}\;&\textbf{if } c'(q_g)\ \text{blocked} \textbf{ then return FAIL} \\[6pt]
\textbf{4.}\;&\mathrm{ThetaStarSearch}(\texttt{cancel})
\qquad\text{（Alg. 2；§3.2–§3.5）} \\[6pt]
\textbf{5.}\;&\textbf{if } g(q_g)=\infty \textbf{ then return FAIL} \\[6pt]
\textbf{6.}\;&\mathrm{Path} \leftarrow \mathrm{ReconstructPath}()
\qquad\text{（Alg. 3；§4.3）} \\[6pt]
\textbf{7.}\;&\textbf{return } \mathrm{Path}
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box algorithm-box-phase-b algorithm-box-subroutine">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 2</span>
    <span class="algorithm-box-title">ThetaStarSearch</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{ThetaStarSearch}(\texttt{cancel}) \to g,\,\pi$

  </div>
  <div class="algorithm-box-io" markdown="1">

| 方向 | 符号 | 说明 |
|------|------|------|
| 输入 | $C'$ | 复制 costmap；$q_s,q_g$ 已栅格化 |
| 输入 | $\mathcal{N}$ | 4/8 邻域（`how_many_corners`） |
| 输出 | $g,\,\pi$ | 累积代价与父指针；失败时 $g(q_g)=\infty$ |

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&g(q_s)\leftarrow 0;\;\pi(q_s)\leftarrow q_s;\;
\text{open}\leftarrow\{q_s\}\ \text{（键 } f=g+h\text{）} \\[6pt]
\textbf{2.}\;&\textbf{while } \text{open}\neq\emptyset \textbf{ do} \\[3pt]
&\quad \textbf{if } \texttt{cancel}() \textbf{ then return FAIL} \\[3pt]
&\quad s\leftarrow \arg\min_{n\in\text{open}} f(n);\;
\textbf{if } s\in\text{closed then continue} \\[3pt]
&\quad \text{close } s;\;
\textbf{if } s=q_g \textbf{ then break} \\[3pt]
&\quad p\leftarrow\pi(s);\;
\textbf{foreach } n\in\mathcal{N}(s),\, n\ \text{free do} \\[3pt]
&\quad\;\;\textbf{if } \mathrm{LOS}(p,n) \textbf{ then }
g'\leftarrow g(p)+w_e\|p\!-\!n\|_2+\tau(n);\;\pi(n)\leftarrow p \\[3pt]
&\quad\;\;\textbf{else }
g'\leftarrow g(s)+w_e\|s\!-\!n\|_2+\tau(n);\;\pi(n)\leftarrow s \\[3pt]
&\quad\;\;\textbf{if } g'<g(n) \textbf{ then }
g(n)\leftarrow g';\;\text{open}\leftarrow\text{open}\cup\{n\} \\[6pt]
\textbf{3.}\;&\textbf{return } g,\,\pi
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box algorithm-box-phase-c algorithm-box-subroutine">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 3</span>
    <span class="algorithm-box-title">ReconstructPath</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{ReconstructPath}() \to \mathrm{Path}$

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&\text{cells}\leftarrow[];\;
n\leftarrow q_g \\[6pt]
\textbf{2.}\;&\textbf{repeat } \text{cells.append}(n);\; n\leftarrow\pi(n)
\ \textbf{until } n=q_s;\;
\text{reverse(cells)} \\[6pt]
\textbf{3.}\;&\textbf{foreach } i\text{: }
\mathrm{Path}.\mathrm{poses}\mathrel{+}=\mathrm{mapToWorld}(\text{cells}[i]);
\ \theta_i\leftarrow\mathrm{atan2}(\Delta y,\,\Delta x) \\[6pt]
\textbf{4.}\;&\mathrm{Path}.\mathrm{poses}[\text{last}].\mathrm{pose}\leftarrow q_g
\qquad\text{（位置 + 目标航向）} \\[6pt]
\textbf{5.}\;&\textbf{return } \mathrm{Path}
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box-footer" markdown="1">

**Complexity**：复制 $O(N)$ + 搜索 $O(N\log N\cdot\bar{L})$ + 回溯 $O(P)$。**与其他插件**：路径更短、任意角；计算量高于 NavFn/Dijkstra，无势场重查。见 [03_navfn.md](03_navfn.md)、[04_dijkstra.md](04_dijkstra.md)。

</div>

</div>

### 5.2 配置与调试

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

```cpp
auto path = server->GetPlan(start, goal, "theta_star_planner", cancel_checker);
```

| 字段 | 说明 |
|------|------|
| `how_many_corners` | 4 或 8 连通（§4.2） |
| `allow_unknown` | UNKNOWN 是否可通行 / 参与 LOS |
| `w_euc_cost` / `w_traversal_cost` / `w_heuristic_cost` | $w_e$ / $w_t$ / $w_h$（§3.3–§3.4） |
| `terminal_checking_interval` | 每 N 次扩展检查 `cancel` |

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| 无路径 | $q_g$ 在膨胀/障碍上 | 调整 goal 或 costmap inflation |
| 路径穿墙 | 8-连通对角切角 | `how_many_corners=4` 或换 NavFn |
| 路径仍锯齿 | LOS 链短、权重偏 $w_t$ | 增大 $w_e$，减小 $w_t$ |
| 规划慢 | LOS 检测 × 堆搜索 | 缩小地图或换 NavFn A* |

**调参**：更短路径 → 增大 $w_e$；更安全 → 增大 $w_t$；窄通道 → `how_many_corners=4`。

---

## 6. 参考文献

1. Nash, A.; Daniel, K.; Koenig, S.; Felner, A., "Theta*: Any-Angle Path Planning on Grids", *AAAI*, 2007. [PDF](https://cdn.aaai.org/AAAI/2007/AAAI07-187.pdf)
2. Hart, P.E.; Nilsson, N.J.; Raphael, B., "A Formal Basis for the Heuristic Determination of Minimum Cost Paths", *IEEE Trans. SSC*, 1968. [DOI:10.1109/TSSC.1968.300136](https://doi.org/10.1109/TSSC.1968.300136)
3. Ferguson & Stentz, "The Field D* Algorithm…", *ICRA*, 2005. — 任意角栅格规划谱系
4. [nav2_theta_star_planner](https://github.com/ros-navigation/navigation2/tree/main/nav2_theta_star_planner) · [§0 指南](00_guide.md) · [§6 综述](06_survey.md)
