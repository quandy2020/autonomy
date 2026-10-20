(theta-star-planner)=
# 5. Theta* 全局规划器

插件 id：`theta_star_planner`。栅格 A* 加上 **Line-of-Sight**，允许任意角折线，路径往往比 NavFn 更短更直。

## 何时用

开阔环境、通道够宽、希望减少锯齿。窄走廊、对角切角风险高时设 `how_many_corners=4`（4 连通）。

## 行为要点

$f=g+h$，$h=w_h\|n-q_g\|_2$。扩展邻居 $n$ 时，若 $\mathrm{LOS}(\pi(s),n)$ 则尝试从祖父直连；否则标准 A* 经 $s$ 步进。

通行代价 $\tau(n)=w_t\cdot c_n/252$。LOS 被 253/254 或（255 且 `!allow_unknown`）阻挡。

| 参数 | 默认 | 含义 |
|------|------|------|
| $w_e$ / $w_t$ / $w_h$ | 2 / 1 / 1 | 段长 / 栅格代价 / 启发 |
| `how_many_corners` | 8 | 4 或 8 连通 |

**没有** NavFn 式终点容差：目标格必须可通行。

## 配置

`theta_star` 块 / `planner.lua` 中权重与连通性。

## 常见坑

| 现象 | 处理 |
|------|------|
| NavFn 有路径 Theta* 无 | 目标在 unknown/lethal；Theta* 更严 |
| 贴角切墙 | 改 4 连通；增大 inflation |
| 算力升高 | LOS 检查次数随扩展增加 |
