(navfn-planner)=
# 3. NavFn 全局规划器

插件 `navfn_planner`。在 Costmap 上建势场 $\phi$，自终点梯度跟踪至起点。

### 定义

Eikonal：$\|\nabla\phi\|=F$，$\phi(q_s)=0$。栅格平面波近似。

代价映射：$F_{ij}=C_n+\kappa_F c_{ij}$（$C_n=50$，$\kappa_F=0.8$）；障碍 `COST_OBS=254`；未访问 $\phi_{\mathrm{HIGH}}=10^{10}$。

平面波更新（4-邻域，$t_a=\min(\phi_u,\phi_d)$，$t_c=\min(\phi_l,\phi_r)$，$\Delta_c=|t_c-t_a|$，$h=F_n$）：

$$
\phi_n=\begin{cases}t_a+h & \Delta_c\ge h\\ t_a+h\cdot v(\Delta_c/h) & \text{else}\end{cases}
$$

$v(r)=-0.2301 r^2+0.5307 r+0.7040$。

### 传播

| 模式 | 配置 | 键 |
|------|------|-----|
| Dijkstra（默认） | `use_astar=false` | $f=\phi$ |
| A* | `use_astar=true` | $f=\phi+\|n-q_g\|_2 C_n$ |

桶宽 $\Delta T=2C_n$；`atStart` 时 $\phi(q_g)$ 可达即停。

路径提取：$\mathbf{p}_{k+1}=\mathbf{p}_k-\delta\nabla\phi/\|\nabla\phi\|$（$\delta=0.5$ 格）。终点不可达时在 $\varepsilon$ 容差内搜 $q^*$（默认 0.1 m）。

### 用法约定

代码：`setGoal(q_s)`、`setStart(q_g)`（与用户语义对调）。流程：复制 costmap → 播种 → 传播 → ResolveGoal → ExtractPath。

配置：`tolerance`、`use_astar`、`allow_unknown`、`use_final_approach_orientation`。
