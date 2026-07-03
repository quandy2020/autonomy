(dijkstra-planner)=
# 4. Dijkstra 全局规划器

> 归属 [NavFn 势场](03_navfn.md) · [§6.5.1 经典图搜索](06_survey.md#651-经典图搜索) · Autonomy ✅ `dijkstra_planner`
>
> **DijkstraPlanner** 继承 `NavfnPlanner`，构造时强制 `SetUseAstar(false)`，排序键 $f(n)=\phi(n)$（$h\equiv 0$）。传播与路径提取走同一 `calcNavFnDijkstra` 代码路径；独立插件用于配置隔离与确定性回归基线。

---

## 1. 背景

**单源最短路径.** Dijkstra（1959）在非负权图上按标号 $d$ **非递减**永久标定节点，保证 SSSP 最优。栅格规划中，代价地图将自由空间离散为 4-连通图；边权有界且为整数时，Dial（1969）以**单调桶队列**将复杂度降至 $O(E+WC)$，$W$ 为最大边权上界。

**NavFn 系实现.** Eriksson & Borenstein（1990）GURVEY 以平面波更新近似 Eikonal $\|\nabla\phi\|=F$（精确解见 Sethian FMM, 1996）。ROS / nav2 的 `navfn` 用 Dial 式桶队列传播 + 梯度跟踪；Autonomy `dijkstra_planner` **显式锁定**该流水线的 Dijkstra 分支（§2），平面波与路径细节见 [03_navfn.md](03_navfn.md)。

---

## 2. 问题

**任务.** 给定 $C$ 与 $q_s,\, q_g \in \mathcal{C}_{\mathrm{free}}$，求 $\tau(0)=q_s$，$\tau(1)\approx q_g$，最小化沿路径通行代价（与 [NavFn §2](03_navfn.md#2-问题) 同型）。

**输入 / 输出.** `DijkstraPlanner::CreatePlan` 继承 `NavfnPlanner::CreatePlan`；输出 `planning_msgs::Path` 及 `SUCCESS` / `NO_PATH` / `CANCELED`。

**在线形式.** 全局规划通常 1–5 Hz 重规划；$h\equiv 0$ 时扩展顺序仅由 $\phi$ 决定，**同输入同地图下输出确定性**，适合作 NavFn 系回归基线。势场约定（与 NavFn 一致，[架构 §1.3.1](01_architecture.md#131-navfn-势场约定)）：$\phi(q_s)=0$（`setGoal`），传播终止于 $\phi(q_g)<\phi_{\mathrm{HIGH}}$（`setStart`，`atStart=true`），路径自 $q^*$ 梯度跟踪至 $q_s$。

**插件继承与配置.**

```
GlobalPlanner
    └── NavfnPlanner          ← use_astar 可选
            └── DijkstraPlanner  ← 强制 SetUseAstar(false)
```

| 项 | `dijkstra_planner` | `navfn_planner`（Dijkstra 模式） |
|----|-------------------|----------------------------------|
| 传播 | `calcNavFnDijkstra` | `use_astar=false` 时相同 |
| 配置域 | `options.dijkstra()` | `options.navfn()` |
| `use_astar` | 不存在（构造时禁用） | 可误改为 `true` |
| 典型用途 | 确定性基线 / 默认插件 ID | 可选切换 A* |

```cpp
DijkstraPlanner::DijkstraPlanner(...) : NavfnPlanner(...) {
    SetUseAstar(false);  // f = g = φ，h ≡ 0
}
```

---

## 3. 图松弛与 Dial 桶队列

章首路线：Dijkstra 松弛 → NavFn 势场对偶 → Dial 桶队列（`propNavFnDijkstra`）。$F_{ij}$ 映射与 `updateCell` 见 [NavFn §2.2–§3.3](03_navfn.md#32-代价映射)。

### 3.1 Dijkstra 松弛（1959）

$$
d(v) = \min_{(u,v)\in E} \big\{ d(u) + w(u,v) \big\}, \qquad d(q_s)=0.
$$

- **$d(v)$**：从 $q_s$ 到 $v$ 的最小累积代价。
- **含义**：非负权下按 $d$ 非递减扩展；label-setting 保证每次弹出节点已最优。

### 3.2 栅格势场对偶

NavFn 在 4-连通栅格上求 $\phi$，连续极限满足 $\|\nabla\phi\|=F$，$\phi(q_s)=0$。$\phi(q)$ 为从 $q$ 到 $q_s$ 的最小累积代价；在**平面波离散更新**下，$\phi$ 与经典 $d$ **近似**共享同一最短路径结构（非严格图论等价；精确 Eikonal 见 FMM）。

| 方面 | 经典 Dijkstra | NavFn Dijkstra（本插件） |
|------|--------------|--------------------------|
| 源点 | $d(q_s)=0$ | $\phi(q_s)=0$（`initCost`） |
| 更新 | 边松弛 | 平面波 `updateCell` |
| 优先级 | 二叉堆 | Dial 式 `curP/nextP/overP` |
| 终止 | $d(q_g)$ 确定 | $\phi(q_g)<\phi_{\mathrm{HIGH}}$ |
| 路径 | 前驱回溯 | `calcPath` 梯度跟踪 |

### 3.3 Dial 桶队列传播

Dial（1969）对整数权 SSSP 按距离分桶，均摊 $O(1)$ 入队。NavFn 取 $\Delta T=2C_n=100$（`priInc`），三缓冲语义：

- **`curP`**：当前 $curT$ 下待更新格点；
- **`nextP`**：$\phi(n)<curT$ 的新 frontier；
- **`overP`**：溢出桶，$curT\leftarrow curT+\Delta T$ 后提升为 `curP`。

`setupNavFn` 在 $q_s$ 播种后，主循环批量 `updateCell` → 交换 `curP/nextP` → 必要时提升 `overP`。永久标定（$\phi$ 不再下降）的格点不再入队；`atStart=true` 时 $\phi(q_g)<\phi_{\mathrm{HIGH}}$ 即停。因 $F_{ij}$ 有界，工程上近 $O(N)$，$N=n_x n_y$；**实际访问格数通常小于 $N$**，但上界仍为 $O(N)$，且 frontier 可能略超出 $q_g$ 最短测地邻域。`cycles=\max(n_x n_y/20,\,n_x+n_y)` 为安全上限。

路径提取与容差搜索同 NavFn，见 [NavFn §4.3](03_navfn.md#43-梯度跟踪与目标容差)。相对 A*（Hart et al., 1968）：本插件 $h\equiv 0$；`navfn_planner` + `use_astar=true` 用 $h=\|n-q_g\|_2\cdot C_n$ 加速（[NavFn §4.2](03_navfn.md#42-a-启发式传播)），可采纳 $h$ 下与 Dijkstra 同解。对比见 [06_survey.md §6.5](06_survey.md#651-经典图搜索)。

---

## 4. 求解

§3 给出 Dijkstra 传播语义；以下 **算法 1** 为 `DijkstraPlanner` 特有流程（步骤 4 固定 Dijkstra）。**算法 2–3**（`PropagateDijkstra` / `ExtractPath`）与 [NavFn §5.1 算法 2–3](03_navfn.md#51-算法数学描述) 相同，Alg. 2 无 A* 初阈分支。

### 4.1 算法（数学描述）

<div class="algorithm-box-diagram">

<div class="algorithm-box algorithm-box-phase-a">
  <div class="algorithm-box-header">
    <span class="algorithm-box-badge">算法 1</span>
    <span class="algorithm-box-title">DijkstraPlanner 全局规划</span>
  </div>
  <div class="algorithm-box-sub" markdown="1">

$\mathrm{CreatePlan}(q_s,\, q_g,\, C;\, \Theta,\, \texttt{cancel}) \mapsto \mathrm{Path}\ \|\ \text{FAIL}\ \|\ \text{CANCELED}$

  </div>
  <div class="algorithm-box-io" markdown="1">

| 方向 | 符号 | 说明 |
|------|------|------|
| 输入 | $q_s,\, q_g$ | 起点 / 终点 `PoseStamped` |
| 输入 | $C$ | 全局 Costmap2D（加锁复制快照） |
| 输入 | $\Theta$ | `tolerance`, `allow_unknown`, `use_final_approach_orientation` |
| 输入 | `cancel` | 传播取消回调（NavFn Alg. 2） |
| 输出 | $\mathrm{Path}$ | 世界坐标航点（$q_s \to q^*$） |
| 输出 | 状态码 | `SUCCESS` / `NO_PATH` / `CANCELED` |

  </div>
  <div class="algorithm-box-body" markdown="1">

$$
\begin{aligned}
\textbf{1.}\;&\text{worldToMap}(q_s);\;
\text{加锁复制 } C \to C';\;
c'(q_s)\leftarrow \text{FREE};\;
\text{setCostmap}(C')
\qquad\text{（NavFn §2.2）} \\[6pt]
\textbf{2.}\;&\text{worldToMap}(q_g);\;
\text{若任一步越界则 return FAIL} \\[6pt]
\textbf{3.}\;&\texttt{setGoal}(q_s);\;\texttt{setStart}(q_g)
\qquad\text{（}\phi(q_s)=0\text{；终止于 } q_g\text{）} \\[6pt]
\textbf{4.}\;&\mathrm{PropagateDijkstra}(\texttt{cancel},\, \texttt{atStart=true})
\qquad\text{（NavFn Alg. 2；§3.3）} \\[6pt]
\textbf{5.}\;&q^* \leftarrow \mathrm{ResolveGoal}(q_g,\, \varepsilon)
\qquad\text{（NavFn §4.3）} \\[6pt]
\textbf{6.}\;&\textbf{if not found}(q^*) \textbf{ then return FAIL} \\[6pt]
\textbf{7.}\;&\mathrm{Path} \leftarrow \mathrm{ExtractPath}(q^*)
\qquad\text{（NavFn Alg. 3）} \\[6pt]
\textbf{8.}\;&\mathrm{smoothApproachToGoal}(q^*,\, \mathrm{Path}) \\[6pt]
\textbf{9.}\;&\textbf{if } \texttt{use_final_approach_orientation} \textbf{ then }
\text{修正末端航向} \\[6pt]
\textbf{10.}\;&\textbf{return } \mathrm{Path}
\end{aligned}
$$

  </div>
</div>

<div class="algorithm-box-footer" markdown="1">

**子程序**：势场传播与路径提取见 [NavFn 算法 2–3](03_navfn.md#51-算法数学描述)。**Complexity**：$O(N)$ 复制 + $O(N)$ 或更小传播 + $O(L)$ 梯度跟踪。**等价性**：与 `navfn_planner`（`use_astar=false`）共用 `calcNavFnDijkstra`，同输入同地图下同输出；配置域不同（§2 表）。

</div>

</div>

### 4.2 配置与调用

```lua
dijkstra_planner = {
    tolerance = 0.1,
    allow_unknown = false,
    use_final_approach_orientation = false,
},
default_planner_id = "dijkstra_planner",
```

```cpp
auto path = server->GetPlan(start, goal, "dijkstra_planner", cancel_checker);
```

| 字段 | 说明 |
|------|------|
| `tolerance` | 目标容差 $\varepsilon$（米）；$\phi(q_g)$ 不可达时在邻域内求 $q^*$ |
| `allow_unknown` | UNKNOWN（255）是否视为可通行 |
| `use_final_approach_orientation` | 末端航向沿路径切向 |

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| 无路径 | $\phi(q^*)\ge\phi_{\mathrm{HIGH}}$ | 增大 `tolerance`，检查 inflation / `allow_unknown` |
| 锯齿路径 | 4-连通 + 梯度离散化 | [Theta*](05_theta_star.md) 或后处理平滑 |
| 与 NavFn Dijkstra 不一致 | 使用了 `navfn_planner` 且 `use_astar=true` | 核对插件 ID 与 `use_astar` |
| 规划慢 | 可达 frontier 大 | `navfn_planner` + `use_astar=true` |

---

## 5. 参考文献

1. Dijkstra, E.W., "A Note on Two Problems in Connexion with Graphs", *Numerische Mathematik*, 1(1):269–271, 1959. [DOI:10.1007/BF01386390](https://doi.org/10.1007/BF01386390)
2. Dial, R.B., "Algorithm 360: Shortest-Path Forest with Topological Ordering", *Communications of the ACM*, 12(11):632–633, 1969. [DOI:10.1145/363269.363610](https://doi.org/10.1145/363269.363610)
3. Hart, P.E.; Nilsson, N.J.; Raphael, B., "A Formal Basis for the Heuristic Determination of Minimum Cost Paths", *IEEE Trans. SSC*, 4(2):100–107, 1968. [DOI:10.1109/TSSC.1968.300136](https://doi.org/10.1109/TSSC.1968.300136)
4. Eriksson & Borenstein, "The GURVEY…", *IEEE AES Magazine*, 1990. [IEEE](https://ieeexplore.ieee.org/document/67314)
5. Sethian, J.A., "A Fast Marching Level Set Method…", *PNAS*, 93(4):1591–1595, 1996. [DOI:10.1073/pnas.93.4.1591](https://doi.org/10.1073/pnas.93.4.1591)
6. Ferguson & Stentz, "The Field D* Algorithm…", *ICRA*, 2005. — 栅格加权最短路径工程谱系
7. LaValle, *Planning Algorithms*, 2006, Ch. 2. [lavalle.pl/planning](https://lavalle.pl/planning/)
8. [NavFn 规划器](03_navfn.md) · [nav2_navfn_planner](https://github.com/ros-navigation/navigation2/tree/main/nav2_navfn_planner) · [§6 综述](06_survey.md)
