# Fields2Cover 参数与算法选择

[返回目录](README.md) · [使用方式](fields2cover_usage.md) · [故障定位](fields2cover_testing.md)

本页的默认值以 [Options](../planner/fields2cover/fields2cover.hpp)、[Robot](../planner/fields2cover/types/robot.hpp) 和对应实现为准。所有长度在示例中使用米，角度使用弧度。

## Robot 基础参数

```cpp
namespace f2c = autonomy::planning::planner::fields2cover;
f2c::F2CRobot robot(1.5, 3.0);  // 车宽 1.5 m，工具幅宽 3 m
robot.SetMinTurningRadius(1.0);
robot.SetCruiseVel(1.0);
robot.SetTurnVel(0.5);
```

- `width` / `SetWidth()`：车体名义宽度。顶层恒定地头的距离由它计算。
- `cov_width` / `SetCovWidth()`：工具作业宽度及相邻条带间距。构造时传 0 会使用车宽；负值构造会抛出 `out_of_range`。
- `SetMinTurningRadius(r)`：设置曲率倒数。实现取绝对值并加数值偏移，应用仍应要求有限且正的半径。
- `SetMaxCurv(k)`：直接设置最大曲率，与设置半径控制同一变量；两者最后一次设置生效。
- `SetMaxDiffCurv(dk)`：曲率变化率，连续曲率算法使用。构造默认 `max_curv=1.0`、`max_diff_curv=0.3`。
- `SetCruiseVel()`：作业和地头直行的速度参数。
- `SetTurnVel()`：转弯速度参数。未设置时 `GetTurnVel()` 返回巡航速度。受约束搜索生成的转弯原语使用该值。

宽度 setter 没有完整输入验收。条带生成器会拒绝非正或非有限作业宽度；调用方仍需提前检查车宽、半径、角度和速度等所有输入。图形规划不检查底盘加速度、工具动力学或完整车体包络。

## Options 的默认值与实际作用

### 地头

- `hg_alg = HGAlg::CONST`：恒定地头。
- `hg_swaths = 3`：地头条数；恒定地头距离为 `robot.GetWidth() * hg_swaths`。
- `HGAlg::NONE`：不内缩，核心区和连接参考都用原始田地。

例如车宽 1.5 m、作业宽 3 m、默认 3 条地头，对应内缩 **4.5 m**。此前随机验证使用 3 m 地头；在相同车宽下，顶层 API 可设置 `hg_swaths=2`。需要任意米制距离时用 `ConstHL::GenerateHeadlands(cells, distance_m)` 分阶段规划。

`CONST` 分支会访问生成的地头环数组，不能传零或负的地头条数。地头小于车辆转弯所需空间时，路径连接可能失败；更大的半径往往要求更多地头空间。扩大地头也会减少核心作业面积，应同时观察核心面积和覆盖率。

### 条带角度与评分

- `sg_alg = SGAlg::BRUTE_FORCE`：搜索方向。
- `sg_obj = SGObjFunc::N_SWATH_MOD`：以边界近似条带数量评分方向，是默认快速目标。
- `sg_angle = 0.0`：只有 `GIVEN_ANGLE` 分支使用。
- `N_SWATH`：最小化实际生成的条带数量。
- `SWATH_LENGTH`：最小化条带长度总和。
- `FIELD_COV`：最大化真实作业带并集覆盖比例。
- `OVERLAPS`：最小化该实现计算的重叠指标。

方向评分使用统一的最小化符号包装；最大化目标会转成负评分。默认 `BruteForce` 角度步长为 1°，实际传参用弧度。要改变步长，直接使用 `BruteForce::SetStepAngle()`，顶层 `Options` 没有相应字段。

无论方向目标是什么，生成条带时都会尝试固定的 5 个相位：`0.5 / 0.125 / 0.375 / 0.625 / 0.875` 个作业宽度，以真实覆盖选择较好起点。相邻条带间距保持一个作业宽度。`GenerateBestSwaths()` 的初选覆盖低于 98% 时还会尝试其他方向，这意味着最后方向可能偏离原目标的最佳方向；给定角分支仅选择相位，不进行此方向补偿。

不要把 98% 触发值当作返回结果验收。应用应自行设置目标覆盖率并独立检查。

### 路线与分解

- `rp_alg = RPAlg::SHORTEST_ROUTE`：当前唯一枚举值；顶层使用 `RoutePlannerBase`。
- `decomp_alg = DecompAlg::NONE`：当前唯一可选顶层分解策略。
- `decomp_angle = 0.0`：目前没有连接到顶层执行分派。

分阶段 `RoutePlannerBase::GenRoute()` 可调整 `d_tol`、条带重定向和优化时间。默认 `d_tol=1e-4`、`redirect_swaths=true`、`time_limit_seconds=1`。当前最近邻 + 2-opt 求解未使用日志和 guided-local-search 参数；时间参数只约束相应路线改进阶段，不是整个覆盖规划的硬超时。

### 转弯

- `pp_alg = PPAlg::DUBINS`：仅前进的有界曲率转弯。
- `DUBINS_CC`：前进的连续曲率转弯，需要曲率变化率参数。
- `REEDS_SHEPP`：允许前进和后退，路径方向信息必须送到支持倒车的执行器。
- `REEDS_SHEPP_HC`：允许换向的 HC 转弯实现，连续曲率能力以对应类声明为准。

本次随机验证覆盖普通 Dubins、普通 Reeds–Shepp 和顶层默认 Dubins 路径；不包含 CC/HC 的 600 场景批量验证。连续曲率候选会进行几何检查，但失败后没有连续曲率约束搜索替代。

`TurningBase::SetDiscretization()` 默认 0.01 m，用于该转弯规划器的离散输出。约束搜索的运动原语有自己的采样规则，并不是所有路径状态都受这个参数统一控制。

## 连接几何的进阶参数

下列 setter 位于 `Robot`，影响 [path_planning.cpp](../planner/fields2cover/path_planning/path_planning.cpp) 的直接跳转、折线圆滑或折叠转角判定：

- `SetMaxCornerCut()`：允许圆滑路径偏离原连接折线的距离；不会免除真实田地检查。
- `SetDirectHopMaxDev()`：将近似直线连接视为直接转弯的偏离阈值。
- `SetUturnMaxHop()`：判定短距离回转的最大距离。
- `SetTrackSimplifyTol()`：连接折线简化距离。
- `SetReversalSweep()`、`SetTurnSlack()`、`SetMinSweep()`：回转、累计转角和有效拐角的角度阈值。
- `SetLegShare()`、`SetRadiusMargin()`、`SetMinBackoffRadii()`、`SetUturnReachRadii()`、`SetMaxCornerSpan()`：控制圆滑候选的几何空间和跨越转角数。

这些参数优化候选形状，不是额外作业覆盖的承诺。先核对输入与基础宽度、半径、地头，再根据具体失败连接调节；每次调节后重新校验覆盖和边界。

## 内部检查、搜索预算与外部验收

当前 `detail::ValidPath()` 对候选连接检查：XY、角度、长度有限；段长非负；每段在原始田地外环内且不穿孔洞；段连接误差不超过 0.01 m；同方向相邻状态的采样曲率不超过 `1.01 / radius`。

约束搜索最多扩展 100,000 个节点，并每 256 次扩展检查约 15 s 的时间预算。二维引导图构建和其他规划阶段不在这个计时检查之内，所以它不是整次 API 调用的硬 15 s 超时。普通 Reeds–Shepp 搜索允许后退，Dubins 搜索只前进。

失败异常包括：

```text
swath width must be positive and finite
No collision-free continuous-curvature connection
No collision-free bounded-curvature connection within search budget
```

独立随机验证的 95% 覆盖率、1 cm 边界容差、5 cm 连接容差、`1.02/r` 曲率容差属于测试验收标准；它们与上述内部连接检查是不同的检查层。具体报告见[验证说明](fields2cover_testing.md)。
