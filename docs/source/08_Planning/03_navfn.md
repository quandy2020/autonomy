(navfn-planner)=
# 3. NavFn 全局规划器

插件 id：`navfn_planner`。在 costmap 上建离散势场 $\phi$，从 **用户终点**（代码里作势场种子）沿梯度走到 **用户起点**。

## 何时用

室内 2D 默认选择；地图较大时可开 A* 减少扩展格数。需要任意角更直路径时换 Theta*。

## 数学要点

Eikonal：$\|\nabla\phi\|=F$，$\phi(q_s)=0$（$q_s$ 为搜索种子端）。栅格用平面波近似更新。

代价：$F_{ij}=C_n+\kappa_F c_{ij}$（$C_n=50$，$\kappa_F=0.8$）；LETHAL=254；未访问 $\phi_{\mathrm{HIGH}}=10^{10}$。

传播模式：

| 模式 | 配置 | 优先级键 |
|------|------|----------|
| Dijkstra 式（默认） | `use_astar=false` | $f=\phi$ |
| A* | `use_astar=true` | $f=\phi+\|n-q_g\|_2 C_n$ |

路径提取：沿 $-\nabla\phi$ 步进（步长约 0.5 格）。终点不可达时在容差内搜最近可达格（默认约 0.1 m）。

## API 约定（易错）

代码调用：`setGoal(用户起点)`、`setStart(用户终点)` — 与日常「起点/终点」语义 **对调**。流程：复制 costmap → 播种 → 传播 → ResolveGoal → ExtractPath。

## 配置

`tolerance`、`use_astar`、`allow_unknown`、`use_final_approach_orientation`（见 planner conf / proto）。

## 常见坑

| 现象 | 处理 |
|------|------|
| 路径锯齿 | 开平滑后处理；或换 Theta* |
| 绕远 | 膨胀过大；或开 A* 仍受势场形状限制 |
| 总是 TIMEOUT | costmap 尺寸与 `isCurrent`；不是 NavFn 独有 |
