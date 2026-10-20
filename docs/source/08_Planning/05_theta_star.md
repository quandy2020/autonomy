(theta-star-planner)=
# 5. Theta* 全局规划器

插件 `theta_star_planner`：栅格 A* + Line-of-Sight，任意角折线。

### 定义

$f=g+h$，$h=w_h\|n-q_g\|_2$。扩展 $s$、父 $p=\pi(s)$、邻居 $n$：

- $\mathrm{LOS}(p,n)$：$\pi(n)\leftarrow p$，$g\leftarrow g(p)+w_e\|p-n\|_2+\tau(n)$
- 否则：标准 A* 步进经 $s$

通行：$\tau(n)=w_t\cdot c_n/252$。LOS 阻塞：253/254，或 255 且 `!allow_unknown`（Bresenham）。

| 参数 | 默认 | 含义 |
|------|------|------|
| $w_e$ / $w_t$ / $w_h$ | 2 / 1 / 1 | 段长 / 代价 / 启发 |
| `how_many_corners` | 8 | 4 或 8 连通 |

**无** NavFn 式终点容差：$q_g$ 须可通行。

### 用法

开阔短路径首选；窄廊用 4-连通防对角切角。配置见 `theta_star` 块 / planner.lua。
