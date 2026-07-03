(navfn-planner)=
# 3. NavFn 全局规划器

> 归属 [§6.5.2 NavFn](06_survey.md#652-navfn-导航势场) · [§0.4 代价地图](00_guide.md#04-代价地图) · Autonomy ✅ `navfn_planner`
>
> **Navigation Function**（NavFn）在全局 Costmap2D 上求解离散导航势场 $\phi$，以**平面波更新**近似 Eikonal 方程；势场自起点 $q_s$ 向外传播后，从 $q_g$ 沿势场下降方向梯度跟踪至 $q_s$，提取几何路径。Autonomy 默认全局规划器，实现于 `autonomy/planning/planner/navfn/`。

---

## 1. 背景

全局规划需在已知栅格地图上，计算从 $q_s$ 到 $q_g$ 的无碰撞几何路径。经典 **Dijkstra / A\*** 在显式图上搜索，边权固定；**导航函数**（Navigation Function）则先建立全图势场 $\phi(q)$，再一次性提取路径，便于重复查询与梯度跟踪。Eriksson & Borenstein（1990）的 GURVEY 与 ROS `navfn` 采用**平面波更新**代替精确 Fast Marching，以桶队列 Dijkstra 实现近线性传播；Navigation2 `nav2_navfn_planner` 与 Autonomy `NavfnPlanner` 均继承该工程化方案（详见 §3–§5）。

---

## 2. 问题

**任务.** 给定全局代价地图 $C$ 与位姿 $q_s,\, q_g \in \mathcal{C}_{\mathrm{free}}$，求离散路径 $\tau$ 使 $\tau(0)=q_s$，$\tau(1)\approx q_g$，并最小化沿路径的通行代价积分。

**输入 / 输出.** `NavfnPlanner::CreatePlan` 接收 `PoseStamped` 起终点与 `Costmap2D`；输出 `planning_msgs::Path`（世界坐标航点序列）。内部将 ROS costmap 映射为 NavFn 代价 $F_{ij}$，在栅格上建立 $\phi$ 后做子像素梯度跟踪。

**实现约定.** 势场零点在**用户起点** $q_s$（`setGoal`），传播终止与路径提取起点在**用户终点** $q_g$（`setStart`）；NavFn 内部 `goal`/`start` 与用户 $q_s$/$q_g$ 对调（§5.1 步骤 2）。`calcPath` 自 $q_g$（或容差解 $q^*$）沿势场下降走向 $q_s$。

---

## 3. 导航势场离散模型

以下给出 §4 传播与 §5 路径提取所需的离散化：连续 Eikonal → 代价映射 → 平面波单元更新。

### 3.1 Eikonal 方程与反向传播

连续模型中，势场 $\phi: \mathbb{R}^2 \to \mathbb{R}_{\ge 0}$ 满足 **Eikonal 方程**：

$$
\|\nabla \phi(x,y)\| = F(x,y), \qquad \phi(q_s) = 0.
$$

- **$F(x,y)\ge 0$**：局部通行代价（越大越难通过）。
- **$\phi(q)$**：从 $q$ 沿最小代价到达 $q_s$ 的累积代价；$\phi(q_s)=0$ 为汇点边界。

NavFn **不**直接解 PDE，而在 4-连通栅格上用平面波更新近似（§3.3）。`setupNavFn` 在 $q_s$ 格调用 `initCost(0)` 播种，当 $\phi(q_g)<\phi_{\mathrm{HIGH}}$ 时可提前终止（`atStart=true`）。

### 3.2 代价映射

ROS / Costmap2D 代价值 $c_{ij}\in[0,252]$ 线性映射为 NavFn 内部通行因子（`setCostmap`）：

$$
F_{ij} = C_n + \kappa_F \cdot c_{ij}, \qquad
C_n = 50,\; \kappa_F = 0.8.
$$

| 符号 / 常量 | 值 | 含义 |
|-------------|-----|------|
| `COST_NEUTRAL` | $C_n=50$ | 开放空间基准代价 |
| `COST_FACTOR` | $\kappa_F=0.8$ | costmap 线性缩放 |
| `COST_OBS` | $254$ | 不可通行，不更新 |
| `POT_HIGH` | $\phi_{\mathrm{HIGH}}=10^{10}$ | 未访问 / 不可达标记 |

- **含义**：$F_{ij}$ 即 §3.3 更新中的 $h$；障碍与膨胀区对应 $F\to\infty$（实现为 `COST_OBS`）。

### 3.3 平面波更新（`updateCell`）

对格点 $n$，取 4-邻域势值 $t_a=\min(\phi_u,\phi_d)$，$t_c=\min(\phi_l,\phi_r)$，$\Delta_c=|t_c-t_a|$，$h=F_n$：

$$
\phi_n = \begin{cases}
t_a + h & \Delta_c \ge h \\[6pt]
t_a + h \cdot v\!\left(\dfrac{\Delta_c}{h}\right) & \mathrm{otherwise}
\end{cases}
$$

插值函数（`navfn.cpp` 二次拟合）：

$$
v(r) = -0.2301\,r^2 + 0.5307\,r + 0.7040, \qquad r = \Delta_c/h.
$$

- **含义**：$\Delta_c\ge h$ 时退化为单邻域 Dijkstra 式更新 $t_a+h$；否则用 $v(r)$ 逼近两邻域 Eikonal 解。
- **实现**：仅当 $\phi_n$ 下降时将 4-邻域推入桶队列（§4.1）；`updateCellAstar` 在此基础上叠加启发式（§4.2）。

---

## 4. 传播与路径提取

在 §3 离散模型上，NavFn 提供两种传播模式，共享同一 `updateCell` 核心；势场建立后统一做梯度跟踪。

### 4.1 Dijkstra 桶队列传播

按势值 $\phi$ **非递减**顺序扩展，排序键 $f(n)=\phi(n)$（无启发式）。Autonomy 默认 `calcNavFnDijkstra(cancel,\, \texttt{atStart=true})`：

- **桶阈值**：$\Delta T = 2 C_n = 100$，按 $\phi$ 分桶，均摊 $O(1)$ 入队。
- **提前终止**：`atStart=true` 时，一旦用户终点格 $\phi(q_g)<\phi_{\mathrm{HIGH}}$ 即停止，不必遍历全图。
- **复杂度**：最坏 $O(N)$，$N=n_x n_y$（栅格总数）；可达区域较小时接近 $O(|\mathcal{R}|)$。

与经典 Dijkstra 的图松弛 $d(v)=\min\{d(u)+w\}$ 等价关系见 [04_dijkstra.md](04_dijkstra.md)。

### 4.2 A* 启发式传播

`use_astar=true` 时调用 `calcNavFnAstar`，排序键：

$$
f(n) = g(n) + h(n), \qquad
g(n)=\phi(n),\quad
h(n) = \|n - q_g\|_2 \cdot C_n.
$$

- **$h$**：到用户终点 $q_g$ 的欧氏距离下界（`updateCellAstar` 中相对 NavFn 内部 `start`），**可采纳**；大地图上通常比 Dijkstra 更快触达 $q_g$。
- **实现**：`updateCellAstar` 在 `updateCell` 基础上维护 A* 优先级；Autonomy 通过 `navfn_planner.use_astar` 切换。

| 模式 | 配置 | 排序键 | 插件 |
|------|------|--------|------|
| Dijkstra | `use_astar=false` | $f=\phi(n)$ | `navfn_planner` / `dijkstra_planner` |
| A* | `use_astar=true` | $f=\phi(n)+h(n)$，$h=\|n-q_g\|_2\cdot C_n$ | `navfn_planner` |

### 4.3 梯度跟踪与目标容差

**路径提取.** 势场建立后，`calcPath` 自 $q_g$（或 $q^*$）出发，步长 $\delta=0.5$ 格（`pathStep`），沿势场下降方向积分（等价于 $-\nabla\phi$）：

$$
\mathbf{p}_{k+1} = \mathbf{p}_k - \delta \cdot \frac{\nabla\phi(\mathbf{p}_k)}{\|\nabla\phi(\mathbf{p}_k)\|}.
$$

`gradCell` 用邻域差分近似下降方向（归一化后与上式同向）：

$$
\frac{\partial\phi}{\partial x} \approx \frac{\phi_{i+1,j}-\phi_{i-1,j}}{2}, \quad
\frac{\partial\phi}{\partial y} \approx \frac{\phi_{i,j+1}-\phi_{i,j-1}}{2}.
$$

终止：$\phi(\mathbf{p}_k) < C_n$ 时到达 $q_s$ 邻域，或迭代超过 `max_cycles = 4\max(n_x,n_y)`。

**目标容差.** 若 $\phi(q_g)\ge\phi_{\mathrm{HIGH}}$，在 $\|q-q_g\|_\infty\le\varepsilon$ 内搜索最近可达点：

$$
q^* = \arg\min_{\|q-q_g\|_\infty \le \varepsilon} \|q-q_g\|_2
\quad \text{s.t.}\quad \phi(q) < \phi_{\mathrm{HIGH}}.
$$

默认 $\varepsilon=0.1\,\mathrm{m}$（`tolerance`）。提取成功后可选 `smoothApproachToGoal` 修正末端离散化伪影。

---

## 5. 求解

§4 给出单步数学；以下 **算法 1–3** 描述 `NavfnPlanner::CreatePlan` 完整流水线（`navfn_planner.cpp` / `navfn.cpp`）。

### 5.1 算法（数学描述）

<div class="algorithm-box-diagram">

<div class="algorithm-box algorithm-box-phase-a">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 1</span>
    <span class="algorithm-box-title">NavfnPlanner 全局规划</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{CreatePlan}(q_s,\, q_g,\, C;\, \Theta,\, \texttt{cancel}) \mapsto \mathrm{Path}\ \|\ \text{FAIL}\ \|\ \text{CANCELED}$

  </div>
  <div class="algorithm-box-io" markdown="1">

| 方向 | 符号 | 说明 |
|------|------|------|
| 输入 | $q_s,\, q_g$ | 起点 / 终点 `PoseStamped` |
| 输入 | $C$ | 全局 Costmap2D（加锁复制快照） |
| 输入 | $\Theta$ | `tolerance`, `use_astar`, `allow_unknown`, `use_final_approach_orientation`, … |
| 输入 | `cancel` | 传播取消回调（Alg. 2） |
| 输出 | $\mathrm{Path}$ | `planning_msgs::Path`（$q_s \to q^*$，世界坐标） |
| 输出 | 状态码 | `SUCCESS` / `NO_PATH` / `CANCELED` |

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&\text{worldToMap}(q_s);\;
\text{加锁复制 } C \to C';\;
c'(q_s)\leftarrow \text{FREE};\;
\text{setCostmap}(C')
\qquad\text{（§3.2 映射为 } F_{ij}\text{）} \\[6pt]
\textbf{2.}\;&\text{worldToMap}(q_g);\;
\text{若任一步越界则 return FAIL} \\[6pt]
\textbf{3.}\;&\texttt{setGoal}(q_s);\;\texttt{setStart}(q_g)
\qquad\text{（势场零点 / 传播终止；§2 约定）} \\[6pt]
\textbf{4.}\;&\textbf{if } \texttt{use_astar} \textbf{ then }
\mathrm{PropagateAstar}(\texttt{cancel}) \textbf{ else }
\mathrm{PropagateDijkstra}(\texttt{cancel},\, \texttt{atStart=true})
\qquad\text{（Alg. 2）} \\[6pt]
\textbf{5.}\;&q^* \leftarrow \mathrm{ResolveGoal}(q_g,\, \varepsilon)
\qquad\text{（§4.3；}\phi(q_g)<\phi_{\mathrm{HIGH}}\text{ 则 } q^*=q_g\text{，否则容差格点搜索）} \\[6pt]
\textbf{6.}\;&\textbf{if } q^*=\varnothing \textbf{ then return FAIL} \\[6pt]
\textbf{7.}\;&\mathrm{Path} \leftarrow \mathrm{ExtractPath}(q^*)
\qquad\text{（Alg. 3；逆序 mapToWorld，} q^* \to q_s\text{）} \\[6pt]
\textbf{8.}\;&\mathrm{smoothApproachToGoal}(q^*,\, \mathrm{Path}) \\[6pt]
\textbf{9.}\;&\textbf{if } \texttt{use_final_approach_orientation} \textbf{ then }
\text{修正末端航向} \\[6pt]
\textbf{10.}\;&\textbf{return } \mathrm{Path}
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box algorithm-box-phase-b algorithm-box-subroutine">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 2</span>
    <span class="algorithm-box-title">势场传播</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{PropagateDijkstra}(\texttt{cancel},\, \texttt{atStart}) \;/\; \mathrm{PropagateAstar}(\texttt{cancel}) \to \{\phi_{ij}\}$

  </div>
  <div class="algorithm-box-io" markdown="1">

| 方向 | 符号 | 说明 |
|------|------|------|
| 输入 | $q_s,\, q_g$ | 已通过 `setGoal` / `setStart` 写入 NavFn |
| 输入 | $F_{ij}$ | `costarr`（§3.2） |
| 输入 | `atStart` | Dijkstra 模式：触达 $q_g$ 即停（Autonomy 默认 `true`） |
| 输出 | $\phi$ | `potarr`；失败时 $\phi(q_g)=\phi_{\mathrm{HIGH}}$ |

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&\texttt{setupNavFn()}\;\text{初始化 }\phi\leftarrow\phi_{\mathrm{HIGH}};\;
\text{initCost}(q_s,\,0)\ \text{（四邻域入 } curP\text{）} \\[6pt]
\textbf{2.}\;&\textbf{if A* then }
curT \leftarrow curT + \|q_s-q_g\|_2\cdot C_n
\qquad\text{（§4.2 初阈）} \\[6pt]
\textbf{3.}\;&cycles \leftarrow \max(n_x n_y/20,\, n_x+n_y) \\[6pt]
\textbf{4.}\;&\textbf{for } cycle = 1 \textbf{ to } cycles \textbf{ do} \\[3pt]
&\quad \textbf{if } \texttt{cancel}() \textbf{ then return CANCELED} \\[3pt]
&\quad \textbf{if } curP=\emptyset \land nextP=\emptyset \textbf{ then break} \\[3pt]
&\quad \text{foreach } n\in curP\text{: }
\text{updateCell}(n)\ \text{或}\ \text{updateCellAstar}(n)
\qquad\text{（§3.3；}\phi_n\downarrow\text{ 则 4-邻域入 } nextP/overP\text{）} \\[3pt]
&\quad curP\leftarrow nextP;\; nextP\leftarrow\emptyset \\[3pt]
&\quad \textbf{if } curP=\emptyset \textbf{ then }
curT\leftarrow curT+2C_n;\; curP\leftarrow overP;\; overP\leftarrow\emptyset
\qquad\text{（§4.1 桶阈值）} \\[3pt]
&\quad \textbf{if } \phi(q_g)<\phi_{\mathrm{HIGH}} \textbf{ and (atStart or A*) then break} \\[6pt]
\textbf{5.}\;&\textbf{return } \phi
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box algorithm-box-phase-c algorithm-box-subroutine">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 3</span>
    <span class="algorithm-box-title">ExtractPath 子程序</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{ExtractPath}(q^*) \to \mathrm{Path}\ \|\ \text{FAIL}$

  </div>
  <div class="algorithm-box-io" markdown="1">

| 方向 | 符号 | 说明 |
|------|------|------|
| 输入 | $q^*$ | 容差解（Alg. 1 步骤 5；通常 $q^*=q_g$） |
| 输入 | $\phi$ | 已传播的 `potarr`（Alg. 2 输出） |
| 输出 | $\mathrm{Path}$ | 栅格路径逆序变换为世界坐标（$q^* \to q_s$） |

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&\text{worldToMap}(q^*);\;
\texttt{setStart}(q^*);\;
\mathbf{p}_0\leftarrow q^*;\;
(\Delta x,\,\Delta y)\leftarrow(0,\,0);\;
k\leftarrow 0 \\[6pt]
\textbf{2.}\;&k_{\max} \leftarrow 4\max(n_x,\, n_y) \\[6pt]
\textbf{3.}\;&\textbf{for } i = 1 \textbf{ to } k_{\max} \textbf{ do} \\[3pt]
&\quad \textbf{if } \phi(\mathbf{p}_k) < C_n \textbf{ then append } q_s;\;
\textbf{return } \mathrm{Path}
\qquad\text{（NavFn \texttt{goal} = 用户 } q_s\text{）} \\[3pt]
&\quad \text{record } \mathbf{p}_k \text{ to pathbuf} \\[3pt]
&\quad \textbf{if } \text{8-邻域高势或振荡} \textbf{ then }
\text{hop to min-}\phi \text{ neighbor}
\qquad\text{（栅格回退）} \\[3pt]
&\quad \textbf{else }
\mathbf{g} \leftarrow \text{bilinearInterp}(\texttt{gradCell},\,\Delta x,\,\Delta y);\;
(\Delta x,\,\Delta y) \mathrel{+}= \delta\cdot\mathbf{g}/\|\mathbf{g}\|
\qquad\text{（§4.3; }\delta=0.5\text{）} \\[3pt]
&\quad \text{update cell index from } (\Delta x,\,\Delta y)
\qquad\text{（子像素进位）};\;
k\leftarrow k+1 \\[6pt]
\textbf{4.}\;&\textbf{if } \text{pathbuf empty} \textbf{ then return FAIL} \\[6pt]
\textbf{5.}\;&\textbf{for } i = \mathrm{len}-1 \textbf{ downto } 0\textbf{: }
\mathrm{Path}.\mathrm{poses} \mathrel{+}= \mathrm{mapToWorld}(\text{pathbuf}[i])
\qquad\text{（} q^* \to q_s \text{）} \\[6pt]
\textbf{6.}\;&\textbf{return } \mathrm{Path}
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box-footer" markdown="1">

**Complexity**：复制 costmap $O(N)$ + 传播 $O(N)$ 或 $O(|\mathcal{R}|)$ + 梯度跟踪 $O(L)$。**插件**：`navfn_planner`（可选 A*）；`dijkstra_planner` 强制 Dijkstra 模式，见 [04_dijkstra.md](04_dijkstra.md)。

</div>

</div>

### 5.2 配置与调试

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
| `use_astar` | `true` → 算法 2 A* 分支；`false` → Dijkstra 桶队列 |
| `allow_unknown` | 是否将 UNKNOWN 格视为可通行 |
| `use_final_approach_orientation` | 末端航向沿路径切向 |

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| 无路径 | $\phi(q^*)\ge\phi_{\mathrm{HIGH}}$（势场未到达 $q_g$ 邻域） | 增大 `tolerance`，检查膨胀层与 `allow_unknown` |
| 路径穿墙 | costmap 未更新 | 确认 obstacle / inflation 层 |
| 锯齿严重 | 网格梯度跟踪 | 换 Theta* 或启用后处理平滑 |
| 规划慢 | 全图 Dijkstra | `use_astar = true` |

**与其他插件**：`navfn_planner`（`use_astar=false`）$\equiv$ Dijkstra 模式；`dijkstra_planner` 独立强制 Dijkstra；`theta_star_planner` 任意角路径，见 [05_theta_star.md](05_theta_star.md)。

---

## 6. 参考文献

1. Eriksson & Borenstein, "The GURVEY: An Autonomous Navigation Algorithm Developed for a Hospital Guidance Robot", *IEEE AES Magazine*, 1990. [IEEE](https://ieeexplore.ieee.org/document/67314)
2. [nav2_navfn_planner](https://github.com/ros-navigation/navigation2/tree/main/nav2_navfn_planner) — Navigation2 工程实现
3. [Planning 指南 §0.3](00_guide.md#03-问题形式化) · [§6 算法综述](06_survey.md)
