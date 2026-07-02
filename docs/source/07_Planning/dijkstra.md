# Dijkstra 全局规划器

`DijkstraPlanner` 强制使用 NavFn 的 Dijkstra 传播模式，禁用 A* 启发式。实现位于 `autonomy/planning/planner/dijkstra/`。

> 完整数学推导见 [Planning 指南 · 数学公式](guide.md#planning-math) §4。

## 1. 与 NavFn 的关系

```
GlobalPlanner
    └── NavfnPlanner          ← use_astar 可选
            └── DijkstraPlanner  ← 强制 use_astar=false
```

```cpp
DijkstraPlanner::DijkstraPlanner(...) : NavfnPlanner(...) {
    SetUseAstar(false);  // 核心：f(n) = g(n)，无启发式
}
```

## 2. Dijkstra 数学原理

### 2.1 松弛方程

在加权图 $G=(V,E)$ 上，边权 $w(u,v) \geq 0$，最短路径距离满足：

$$
d(v) = \min_{(u,v) \in E} \big\{ d(u) + w(u,v) \big\}
$$

初始化 $d(q_g) = 0$，其余 $d(v) = \infty$。按 $d$ **非递减**顺序扩展节点，当 $d$ 最小时确定最优，此即 Dijkstra 的正确性证明基础。

### 2.2 NavFn 中的实现

NavFn 将图松弛替换为**平面波更新**（见 [NavFn 文档](navfn.md) §2.1），并用桶队列按 $\phi$ 值分桶：

$$
\text{桶阈值: } T,\; T + 2C_n,\; T + 4C_n,\; \ldots \quad (C_n = 50)
$$

| 方面 | 经典 Dijkstra | NavFn Dijkstra |
|------|--------------|----------------|
| 搜索方向 | $q_s \to q_g$ | $q_g \to q_s$（反向势场） |
| 数据结构 | 二叉堆 $O(\log N)$ | 桶队列 $\approx O(1)$ |
| 边权 | 显式 $w(u,v)$ | 平面波 + $F_{ij}$ |
| 终止 | $d(q_g)$ 确定 | $\phi(q_s) < \texttt{POT\_HIGH}$ |

### 2.3 提前终止

`atStart=true` 时，一旦 $\phi(q_s) < \texttt{POT\_HIGH}$ 即停止：

$$
|\{n : \phi(n) \text{ 已更新}\}| \ll |V| = M \times N
$$

实际复杂度接近 $O(|\text{可达区域}|)$，而非全图 $O(N)$。

## 3. 与 A* 的对比

| 维度 | DijkstraPlanner | NavFn A* |
|------|-----------------|----------|
| 排序键 | $f = g = \phi(n)$ | $f = g + h$ |
| 启发式 | $h \equiv 0$ | $h = \|n-q_s\|_2 \cdot C_n$ |
| 扩展顺序 | 代价递增（广度） | 最佳优先 |
| 确定性 | 高 | 高（可采纳 $h$） |
| 大地图速度 | 较慢 | 通常更快 |

**选择建议**：

- 需要语义明确的 Dijkstra 插件 → `dijkstra_planner`
- 需要更快响应 → `navfn_planner` + `use_astar = true`
- 与 `navfn_planner`（`use_astar=false`）行为等价

## 4. 配置与调用

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

## 5. 复杂度

设 $N = M \times N_{\text{cells}}$，$L$ 为路径长度：

| 阶段 | 时间 | 空间 |
|------|------|------|
| 复制 costmap | $O(N)$ | $O(N)$ |
| Dijkstra 传播 | $O(N)$ 最坏，通常更小 | $O(N)$ |
| 梯度跟踪 | $O(L)$ | $O(L)$ |

## 6. 常见问题

**Q: 与 `navfn_planner`（`use_astar=false`）有何区别？**

A: 算法完全相同。`dijkstra_planner` 是独立插件，配置不会被 `navfn_planner.use_astar` 误改。

**Q: 为何从 goal 反向传播？**

A: 势场 $\phi$ 以 goal 为源只需计算一次；换起点时只需重新梯度跟踪，不必重算全图。

## 7. 相关文档

- [Planning 指南 · 数学公式](guide.md#planning-math)
- [NavFn 规划器](navfn.md)
- [架构设计](architecture.md)
