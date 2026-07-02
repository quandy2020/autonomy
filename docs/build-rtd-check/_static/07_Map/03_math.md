(map-math)=
# 3. 数学原理

> 完整算法对比与工程背景见 [08_survey.md](08_survey.md)；Costmap2D / GridMap 实现细节见 [06_costmap2d.md](06_costmap2d.md)、[07_grid_map.md](07_grid_map.md)。

### 3.1 环境地图的形式化

移动机器人在平面环境 $\mathcal{W} \subset \mathbb{R}^2$ 中导航。环境被离散为栅格集合 $\mathcal{M}$，每个栅格 $c_{ij}$ 携带环境属性（占据概率、通行代价、高程等）：

$$
\mathcal{M} = \{ c_{ij} \mid i \in [0, N_x), \; j \in [0, N_y) \}
$$

分辨率 $\Delta$（m/cell）将连续坐标 $\mathbf{p} = (x, y)^\top$ 映射到离散索引 $(m_x, m_y)$。Autonomy 提供两套坐标约定，见 §3.2 与 §3.5。

### 3.2 Costmap2D 坐标变换

Costmap2D 采用 **左下角原点**（与 Nav2 / ROS `OccupancyGrid` 一致）。

**世界坐标 → 栅格索引**（`worldToMap`）：

$$
m_x = \left\lfloor \frac{x - x_0}{\Delta} \right\rfloor, \quad
m_y = \left\lfloor \frac{y - y_0}{\Delta} \right\rfloor
$$

其中 $(x_0, y_0)$ 为地图左下角世界坐标（`origin_x_`, `origin_y_`）。

**栅格索引 → 世界坐标**（cell 中心，`mapToWorld`）：

$$
x = x_0 + \left(m_x + \frac{1}{2}\right) \Delta, \quad
y = y_0 + \left(m_y + \frac{1}{2}\right) \Delta
$$

**线性存储索引**（行优先）：

$$
\mathrm{idx} = m_y \cdot N_x + m_x
$$

**世界距离 → 栅格数**：

$$
d_{\text{cell}} = \left\lceil \frac{d_{\text{m}}}{\Delta} \right\rceil, \quad d_{\text{cell}} \geq 0
$$

### 3.3 OccupancyGrid 与 Costmap2D 的数值映射

`OccupancyGrid` 使用 $o \in \{-1, 0, \ldots, 100\}$（未知 / 自由 / 占据），Costmap2D 使用 $c \in [0, 255]$。

**正向映射**（`OccupancyGrid` → `Costmap2D`）：

$$
c = \mathrm{round}\!\left( \frac{o \cdot (C_{\mathrm{L}} - C_{\mathrm{F}})}{O_{\mathrm{occ}} - O_{\mathrm{free}}} \right)
$$

其中 $C_{\mathrm{F}}=0$（`FREE`）、$C_{\mathrm{L}}=254$（`LETHAL`）、$O_{\mathrm{free}}=0$、$O_{\mathrm{occ}}=100$。未知格 $o=-1$ 映射为 $C_{\mathrm{NI}}=255$（`NO_INFORMATION`）。

**反向映射**（`snapshotOccupancyGrid`）：

| Costmap 值 | OccupancyGrid |
|------------|---------------|
| `NO_INFORMATION` (255) | -1 |
| `LETHAL_OBSTACLE` (254) | 100 |
| `INSCRIBED_INFLATED_OBSTACLE` (253) | 99 |
| 其他 $c \in [0, 252]$ | $\mathrm{round}(c \cdot 98 / 252)$ |

### 3.4 代价值语义

| 常量 | 值 | 含义 |
|------|-----|------|
| `FREE_SPACE` | 0 | 完全自由 |
| `1 … MAX_NON_OBSTACLE` | 1–252 | 距障碍越近代价越高 |
| `INSCRIBED_INFLATED_OBSTACLE` | 253 | 内切膨胀区（机器人中心不可进入） |
| `LETHAL_OBSTACLE` | 254 | 致命障碍 |
| `NO_INFORMATION` | 255 | 未知 |

规划器将 $c \geq 253$ 视为不可通行（Theta* 等），NavFn 通过内部映射 $F_{ij} = 50 + 0.8 \cdot c_{ij}$ 转化为势场代价。

### 3.5 GridMap 坐标变换

GridMap 采用 **中心坐标系** + **循环 buffer**，与 Costmap2D 约定不同。

**地图尺寸与栅格数**：

$$
N_i = \mathrm{round}\!\left(\frac{L_i}{\Delta}\right), \quad
L_i^{\text{actual}} = N_i \cdot \Delta
$$

**中心到数据原点的向量**：

$$
\vec{v}_{\text{origin}} = \frac{1}{2} \mathbf{L}
$$

**中心到首格中心的向量**：

$$
\vec{v}_{\text{first}} = \vec{v}_{\text{origin}} - \frac{1}{2}\Delta \cdot \mathbf{1}
$$

**位置 → 索引**：

$$
\vec{i}_{\text{vec}} = \frac{\mathbf{p} - \vec{v}_{\text{origin}} - \mathbf{p}_{\text{map}}}{\Delta}
$$

再经 buffer 变换 $T_{\text{buf}} = -\mathbf{I}_2$（`transformMapFrameToBufferOrder`）得到 buffer 索引。

**索引 → 位置**：

$$
\mathbf{p} = \mathbf{p}_{\text{map}} + \vec{v}_{\text{first}} + \Delta \cdot T_{\text{buf}}^{-1}(\mathbf{i}_{\text{unwrapped}})
$$

**循环 buffer 索引换算**：

$$
\mathbf{i}_{\text{unwrapped}} = \mathrm{wrap}(\mathbf{i}_{\text{buffer}} - \mathbf{s}_{\text{start}})
$$

$$
\mathbf{i}_{\text{buffer}} = \mathrm{wrap}(\mathbf{i} + \mathbf{s}_{\text{start}})
$$

其中 $\mathrm{wrap}$ 为模运算，保证索引落在 $[0, N_i)$。

### 3.6 膨胀层（InflationLayer）数学

膨胀将致命障碍 $c=254$ 向外传播，形成代价梯度，使规划器在靠近障碍时自动绕行。

设 $d$ 为**栅格单位**的欧氏距离（到最近障碍种子），$r_i$ 为机器人 footprint **内切圆半径**（`inscribed_radius_`），$\lambda$ 为 `cost_scaling_factor_`（默认 10.0），$R$ 为 `inflation_radius_`（默认 0.55 m）。

**代价函数** `computeCost(d)`：

$$
c(d) =
\begin{cases}
254 & d = 0 \\[6pt]
253 & d \cdot \Delta \leq r_i \\[6pt]
\left\lfloor 252 \cdot e^{-\lambda (d \cdot \Delta - r_i)} \right\rfloor & \mathrm{otherwise}
\end{cases}
$$

其中 $254$、`253` 分别对应 `LETHAL_OBSTACLE`、`INSCRIBED_INFLATED_OBSTACLE`。

**物理解释**：

- $d \cdot \Delta \leq r_i$：机器人中心进入此区域将与障碍碰撞（内切圆模型）
- 指数衰减：距障碍越远代价越低，$\lambda$ 越大衰减越快
- 膨胀半径 $R$ 决定传播范围：$d_{\text{cell}} = \lceil R / \Delta \rceil$

**预计算缓存**（避免实时 `exp`）：

$$
D_{ij} = \sqrt{i^2 + j^2}, \quad C_{ij} = c(D_{ij})
$$

**传播算法**：从所有 lethal（及可选 unknown）种子出发，按整数距离等级 BFS 扩展 4-邻域；`updateBounds` 向外扩展 $R$ 以保证边界格也被正确膨胀。

### 3.7 Footprint 与内切半径

机器人 footprint 为凸多边形 $\mathcal{F} = \{ \mathbf{v}_0, \ldots, \mathbf{v}_{n-1} \}$（机器人坐标系）。

**旋转变换到世界系**：

$$
\begin{bmatrix} x' \\ y' \end{bmatrix} =
\begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}
\begin{bmatrix} x \\ y \end{bmatrix} +
\begin{bmatrix} x_{\text{robot}} \\ y_{\text{robot}} \end{bmatrix}
$$

**内切半径** $r_i$（`calculateMinAndMaxDistances`）：

$$
r_i = \min_{k} \Big\{ \|\mathbf{v}_k\|_2, \; d(\mathbf{0}, \overline{\mathbf{v}_k \mathbf{v}_{k+1}}) \Big\}
$$

**外接半径** $r_c = \max_k \|\mathbf{v}_k\|_2$。

**点到线段距离**（`distanceToLine`）：

$$
d(\mathbf{p}, \overline{\mathbf{a}\mathbf{b}}) =
\begin{cases}
\|\mathbf{p} - \mathbf{a}\| & t < 0 \\
\|\mathbf{p} - \mathbf{b}\| & t > 1 \\
\frac{|(\mathbf{b}-\mathbf{a}) \times (\mathbf{a}-\mathbf{p})|}{\|\mathbf{b}-\mathbf{a}\|} & 0 \leq t \leq 1
\end{cases}
$$

其中 $t = \frac{(\mathbf{p}-\mathbf{a}) \cdot (\mathbf{b}-\mathbf{a})}{\|\mathbf{b}-\mathbf{a}\|^2}$。

### 3.8 障碍层射线清除（Bresenham）

激光 **clearing** 从传感器原点 $\mathbf{o}$ 到障碍点 $\mathbf{p}$ 沿射线将沿途栅格设为 `FREE_SPACE`。

Bresenham 整数算法：设 $\Delta x = |p_x - o_x|$，$\Delta y = |p_y - o_y|$，$s_x = \mathrm{sign}(p_x - o_x)$，$s_y = \mathrm{sign}(p_y - o_y)$，从 $(o_x, o_y)$ 逐步走向 $(p_x, p_y)$，每步更新误差项：

$$
\epsilon = 2 \cdot \Delta y - \Delta x
$$

若 $\epsilon > 0$：$y \leftarrow y + s_y$，$\epsilon \leftarrow \epsilon - 2\Delta x$；否则 $x \leftarrow x + s_x$，$\epsilon \leftarrow \epsilon + 2\Delta y$。

**Marking**：障碍点投影到 2D 后设为 `LETHAL_OBSTACLE`。

### 3.9 Rolling Window（Costmap2D）

局部代价地图以机器人为中心滑动。新原点：

$$
x_0^{\text{new}} = x_{\text{robot}} - \frac{W}{2}, \quad
y_0^{\text{new}} = y_{\text{robot}} - \frac{H}{2}
$$

`updateOrigin` 步骤：

1. 计算栅格偏移 $\delta_x = \lfloor (x_0^{\text{new}} - x_0) / \Delta \rfloor$（对齐到栅格）
2. 求新旧窗口重叠区域，copy 到临时 buffer
3. 全图 reset 为 `default_value_`，更新 origin
4. 重叠数据写回新位置；非重叠区域为未知/自由

### 3.10 GridMap 滑动窗口（move）

GridMap 的 `move(position)` 使用**循环 buffer**，数据在 map frame 中保持 stationary：

$$
\Delta \mathbf{p} = \mathbf{p}_{\text{new}} - \mathbf{p}_{\text{map}}, \quad
\Delta \mathbf{i} = \mathrm{round}(\Delta \mathbf{p} / \Delta)
$$

对齐后 $\Delta \mathbf{p}_{\text{aligned}} = \Delta \mathbf{i} \cdot \Delta$，移出 buffer 的行/列设为 `NaN`，更新 `startIndex_` 与 `position_`。

### 3.11 GridMap 插值

`atPosition(layer, pos, value, method)` 支持四种模式：

| 模式 | 方法 | 精度 | 计算量 |
|------|------|------|--------|
| `INTER_NEAREST` | 最近邻 | 低 | $O(1)$ |
| `INTER_LINEAR` | 双线性（2×2） | 中 | $O(1)$ |
| `INTER_CUBIC_CONVOLUTION` | Keys 1981 双三次卷积 | 高 | $O(1)$ |
| `INTER_CUBIC` | 标准双三次（含导数） | 最高 | $O(1)$ |

**双线性插值**：设查询点落在 unit square $[0,1]^2$，四角值为 $f_{00}, f_{10}, f_{01}, f_{11}$：

$$
f(x,y) = (1-x)(1-y)f_{00} + x(1-y)f_{10} + (1-x)y f_{01} + xy f_{11}
$$

**Keys 双三次卷积 1D**（`convolve1D`）：

$$
y = \frac{1}{2} [1,\; t,\; t^2,\; t^3] \cdot M_{\text{conv}} \cdot \mathbf{f}
$$

其中 $M_{\text{conv}}$ 为 4×4 Keys 矩阵：

$$
M_{\text{conv}} = \begin{bmatrix}
0 & 2 & 0 & 0 \\
-1 & 0 & 1 & 0 \\
2 & -5 & 4 & -1 \\
-1 & 3 & -3 & 1
\end{bmatrix}
$$

**标准双三次**：在 unit square 四角取函数值及偏导 $\partial_x f$, $\partial_y f$, $\partial_{xy} f$（中心差分）：

$$
\frac{\partial f}{\partial x}\bigg|_{i,j} \approx \frac{f_{i+1,j} - f_{i-1,j}}{2\Delta}
$$

组装 4×4 `FunctionValueMatrix`，经 $M_{\text{bicubic}}$ 求多项式系数后求值。

### 3.12 图层合并策略

`CostmapLayer` 写入 master grid 时的合并模式：

| 模式 | 规则 |
|------|------|
| `updateWithMax` | $c_{\text{master}} = \max(c_{\text{master}}, c_{\text{layer}})$；master 为 `NO_INFORMATION` 时被覆盖 |
| `updateWithMaxWithoutUnknownOverwrite` | 未知格不被覆盖 |
| `updateWithOverwrite` | 跳过 layer 中的 `NO_INFORMATION` |
| `updateWithTrueOverwrite` | 全部覆盖 |
| `updateWithAddition` | 相加，上限 `INSCRIBED_INFLATED_OBSTACLE - 1` |

### 3.13 公式速查

| 主题 | 核心公式 | 详见 |
|------|----------|------|
| Costmap 坐标 | $m_x = \lfloor(x-x_0)/\Delta\rfloor$ | [06_costmap2d.md §6.5](06_costmap2d.md) |
| 膨胀代价 | $c = \lfloor 252 \cdot e^{-\lambda(d\Delta - r_i)} \rfloor$ | [06_costmap2d.md §6.8](06_costmap2d.md) |
| GridMap 坐标 | $\vec{i} = (\mathbf{p} - \vec{v}_o - \mathbf{p}_m)/\Delta$ | [07_grid_map.md §7.6](07_grid_map.md) |
| 双三次卷积 | Keys $M_{\text{conv}}$ 矩阵 | [07_grid_map.md §7.8](07_grid_map.md) |
| 占据映射 | $c = \mathrm{round}(o \cdot 254 / 100)$ | §3.3 |
