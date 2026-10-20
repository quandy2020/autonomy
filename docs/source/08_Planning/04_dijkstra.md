(dijkstra-planner)=
# 4. Dijkstra 全局规划器

插件 `dijkstra_planner`：NavFn 强制 Dijkstra 模式（$h\equiv 0$），确定性基线。

### 定义

在栅格图上 SSSP：边权来自平面波 / 邻域代价。与经典 Dijkstra 松弛等价；实现复用 NavFn 桶队列（$\Delta T=2C_n$）。

相对 A*：无启发式，最坏扫更多格，但结果确定、易回归。

### 结构

`DijkstraPlanner` ⊂ `NavfnPlanner`，固定 `use_astar=false`。势场约定同 [§3](03_navfn.md)（`setGoal`/`setStart` 对调）。

### 用法

```lua
default_planner_id = "dijkstra_planner"
```

代价映射、容差、`allow_unknown` 与 NavFn 相同。场景：单元测试、对比基线；生产默认仍推荐 `navfn_planner`（可开 A*）。
