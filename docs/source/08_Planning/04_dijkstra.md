(dijkstra-planner)=
# 4. Dijkstra 全局规划器

插件 id：`dijkstra_planner`。实现上是 NavFn **固定** `use_astar=false`（$h\equiv 0$），用作确定性、可复现的基线。

## 何时用

- 单元测试、算法对比、回归「同一地图同一结果」。
- 生产默认仍推荐 `navfn_planner`（可按需开 A*）。

## 行为

与 [NavFn](03_navfn.md) 共享势场更新与 `setGoal`/`setStart` 对调约定；桶队列宽度 $\Delta T=2C_n$。

## 配置

```lua
default_planner_id = "dijkstra_planner"
```

`tolerance`、`allow_unknown`、代价映射与 NavFn 相同。

## 常见坑

| 现象 | 说明 |
|------|------|
| 比 A* NavFn 慢 | 无启发式，扩展更多格，属预期 |
| 与 NavFn 路径不同 | 传播顺序与 tie-break 可能不同；几何上通常接近 |
