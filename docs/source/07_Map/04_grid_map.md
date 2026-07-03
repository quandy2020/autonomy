# 4. GridMap 2.5D 栅格地图

`GridMap` 是 Autonomy 的**多层浮点栅格地图**实现，源自 ETH Zurich `grid_map`。它在 XY 平面上维护多个命名数据层，通过层值编码高程等地形信息，故称 **2.5D**。

> 模块级架构见 [§1.4](01_architecture.md#14-gridmap-25d-架构重点)；公式见 [§4.6–§4.8](04_grid_map.md#46-坐标变换)。

---

## 4.1 何时使用 GridMap

| 场景 | 是否适用 | 说明 |
|------|----------|------|
| 地形高程建模 | ✅ | `elevation` 层 |
| 坡度/台阶检测 | ✅ | 由高程差分计算 `slope` 层 |
| 路径规划避障 | ❌ 直接用 | 用 [Costmap2D](03_costmap2d.md)；或自行桥接 |
| 激光 2D 障碍 | ❌ | Costmap2D 更合适 |
| 多模态感知融合 | ✅ | 每层独立写入（语义、颜色、方差等） |

**重要**：GridMap 当前是**数据容器 + 几何/插值工具**，没有 Costmap2D 那样的自动传感器流水线。

---

## 4.2 架构：2.5D 心智模型

### 4.2.1 与 2D / 3D 的区别

```
2D OccupancyGrid          2.5D GridMap                3D OctoMap
┌───┬───┐                 ┌───┬───┐                   体素树
│ 0 │100│  每格一个值       │ele│ele│  每格多层 float      稀疏 3D
├───┼───┤                 │var│slp│  elevation          占据概率
│ 0 │ 0 │                 ├───┼───┤  variance           内存大
└───┴───┘                 │...│...│  slope ...
                          └───┴───┘
```

### 4.2.2 对象层级

```
你的代码
  │
  ▼
GridMapWrapper              ← MapInterface 生命周期
  │
  └── grid_map::GridMap     ← 核心：多层 Eigen::MatrixXf
        ├── data_["elevation"]   → Matrix (rows × cols)
        ├── data_["variance"]    → Matrix
        ├── position_            → 地图中心 (x, y) [m]
        ├── length_              → 物理尺寸 (Lx, Ly) [m]
        ├── resolution_          → [m/cell]
        └── startIndex_          → 循环 buffer 起点（move 时用）
```

### 4.2.3 坐标系示意（俯视图）

```
                    +Y (map frame)
                     ↑
         ┌───────────────────────┐
         │                       │
         │    ● position_        │  ← 地图中心（非左下角！）
         │      (地图中心)        │
         │                       │
         └───────────────────────┘
       ← length_x →
    -X ────────────── +X

    cell (0,0) 在 buffer 中，经 T_buf=-I 翻转后与 map frame 对齐
    无效数据 = NaN（不是 Costmap 的 255）
```

### 4.2.4 与 Costmap2D 对比（必读）

| 维度 | Costmap2D | GridMap |
|------|-----------|---------|
| **定位** | 规划/避障的标准输入 | 地形/感知的浮点容器 |
| **数值** | `uint8` 0–255 离散代价 | `float` 连续值 |
| **原点** | 左下角 $(x_0, y_0)$ | 中心 $\mathbf{p}_{\text{map}}$ |
| **更新** | 后台线程 + 插件自动融合 | **应用层手动写层** |
| **传感器** | `feedLaserScan` 等内置 | 无内置 feed |
| **Planning** | ✅ 已集成 | ❌ 需自行桥接 |
| **滑动窗口** | `updateOrigin` 平移 copy | `move()` 旋转 buffer 索引 |

---

## 4.3 详细使用指南

### 4.3.1 最小完整流程

```cpp
#include "autonomy/map/grid_map/grid_map_wrapper.hpp"

// 1. 配置（或手动构造 GridMapOptions）
proto::GridMapOptions opts;
opts.set_resolution(0.1);       // 10 cm/cell
opts.set_length_x(10.0);       // 10 m
opts.set_length_y(10.0);
opts.set_position_x(0.0);      // 地图中心世界坐标
opts.set_position_y(0.0);
opts.add_layers("elevation");
opts.add_layers("variance");
opts.set_frame_id("map");

// 2. 构造 + 启动
auto wrapper = std::make_shared<autonomy::map::grid_map::GridMapWrapper>(opts);
wrapper->Start();

// 3. 获取核心对象
auto& gm = *wrapper->getGridMap();

// 4. 写入数据（应用层负责：点云投影、滤波等）
grid_map::Index idx;
if (gm.getIndex(grid_map::Position(1.0, 2.0), idx)) {
    gm.at("elevation", idx) = 0.35f;   // 35 cm 高程
    gm.at("variance", idx) = 0.01f;
}

// 5. 连续查询（插值）
grid_map::Position query(1.05, 2.03);
double elev;
gm.atPosition("elevation", query, elev,
              grid_map::InterpolationMethods::INTER_LINEAR);

// 6. 机器人移动时滑动窗口
gm.move(grid_map::Position(robot_x, robot_y));

// 7. 停止
wrapper->Stop();
```

### 4.3.2 典型工作流：点云 → 高程层

```mermaid
flowchart LR
    A["PointCloud2"] --> B["坐标变换到 map frame"]
    B --> C["按 XY 分桶<br/>取 Z 最大/平均"]
    C --> D["写入 elevation 层<br/>at() / addDataFrom"]
    D --> E["可选: 高斯平滑<br/>grid_map_cv"]
    E --> F["计算 slope 层<br/>梯度或邻域差分"]
    F --> G["atPosition 查询可通行性"]
```

GridMap **不包含**步骤 B–F 的现成实现，需在应用层或 `grid_map_cv` 中完成。

### 4.3.3 常用 API 速查

| API | 作用 | 备注 |
|-----|------|------|
| `setGeometry(length, res, position)` | 初始化/重置尺寸 | 清空所有层数据 |
| `add("layer_name")` | 新增一层 | 初始为 NaN |
| `at("layer", index)` | 按索引读写 | 直接访问 Matrix |
| `atPosition("layer", pos, value)` | 连续插值查询 | 支持 4 种插值模式 |
| `getIndex(Position, Index&)` | 世界坐标 → 索引 | 越界返回 false |
| `getPosition(Index, Position&)` | 索引 → 世界坐标 | |
| `move(Position)` | 滑动窗口 | 移出区域置 NaN |
| `addDataFrom(other, ...)` | 融合另一张 GridMap | 多传感器累积 |
| `isInside(Position)` | 是否在地图内 | |

### 4.3.4 添加自定义层

```cpp
// 由高程计算坡度层
gm.add("slope");
for (grid_map::GridMapIterator it(gm); !it.isPastEnd(); ++it) {
    const grid_map::Index idx = *it;
    if (std::isnan(gm.at("elevation", idx))) continue;

    grid_map::Position pos;
    gm.getPosition(idx, pos);
    double dz_dx, dz_dy;
    // 中心差分求梯度 ...
    gm.at("slope", idx) = std::atan(std::hypot(dz_dx, dz_dy));
}
```

### 4.3.5 配置参考

```cpp
// proto::GridMapOptions 字段
resolution   = 0.1;      // m/cell
length_x     = 10.0;     // m
length_y     = 10.0;
position_x   = 0.0;      // 地图中心
position_y   = 0.0;
frame_id     = "map";
layers       = ["elevation", "variance"];
basic_layers = ["elevation"];  // 判定有效：elevation 为 NaN 则该 cell 无效
```

Lua 加载：`CreateGridMapOptions(parameter_dictionary)`（`layers` 列表解析部分待完善）。

### 4.3.6 当前限制（务必知晓）

| 功能 | 状态 | 替代方案 |
|------|------|----------|
| 后台自动更新 | ❌ 无线程 | 应用层定时写层 |
| `feedLaserScan` 等 | ❌ | 自行投影点云到 elevation |
| `publishMap()` | ❌ TODO | 自行序列化 `grid_map_msgs` |
| `loadMap()` | ❌ TODO | 手动读文件 + `setGeometry` |
| Costmap 桥接 | ❌ | 应用层将 slope/elevation 映射为 cost |

### 4.3.7 与 Costmap2D 联合使用（应用层桥接）

若需将地形信息纳入规划，可在应用层实现：

```cpp
// 伪代码：将坡度映射为 Costmap 代价
for (each cell in costmap window) {
    double slope;
    if (grid_map.atPosition("slope", world_pos, slope)) {
        if (slope > 0.3)  // ~17°
            costmap.setCost(mx, my, LETHAL_OBSTACLE);
        else if (slope > 0.15)
            costmap.setCost(mx, my, 128);  // 中等代价
    }
}
```

官方 `grid_map_costmap_2d/` 桥接模块代码已注释，尚未启用。

---

## 4.4 2.5D 概念与典型层

传统 2D 栅格每个 cell 只有一个值（占据/代价）。GridMap 每个 cell 可携带**多个浮点属性**：

| 典型层名 | 含义 | 单位 |
|----------|------|------|
| `elevation` | 地面/表面高程 | m |
| `variance` | 高程不确定性 | m² |
| `color` | RGB 编码颜色 | — |
| `surface_normal_x/y/z` | 表面法向量分量 | — |
| `traversability` | 可通行性评分 | 0–1 |

查询 $(x, y)$ 时通过插值得到连续值；查询 3D 位置时使用 `getPosition3` 返回 $(x, y, z=z_{\mathrm{elev}})$，其中 $z_{\mathrm{elev}}$ 为 `elevation` 层值。

## 4.5 核心数据结构

路径：`autonomy/map/grid_map/grid_map_core/grid_map.hpp`

| 成员 | 类型 | 含义 |
|------|------|------|
| `data_` | `unordered_map<string, Matrix>` | 层名 → `Eigen::MatrixXf` |
| `layers_` | `vector<string>` | 层名列表（有序） |
| `basicLayers_` | `vector<string>` | 有效性判定层（含 NaN 则无效） |
| `length_` | `Length (Vector2d)` | 地图物理尺寸 [m] |
| `resolution_` | `double` | 分辨率 [m/cell] |
| `position_` | `Position (Vector2d)` | 地图中心在世界系中的位置 |
| `size_` | `Size (Vector2i)` | buffer 行列数 |
| `startIndex_` | `Index (Vector2i)` | 循环 buffer 起始索引 |

类型定义（`type_defs.hpp`）：

```cpp
using Matrix = Eigen::MatrixXf;
using Position = Eigen::Vector2d;
using Length = Eigen::Vector2d;
using Index = Eigen::Array2i;
```

## 4.6 坐标变换

GridMap 使用**中心坐标系**，与 Costmap2D 的左下角原点不同。

### 4.6.1 几何初始化

$$
N_i = \mathrm{round}\!\left(\frac{L_i}{\Delta}\right), \quad
L_i^{\text{actual}} = N_i \cdot \Delta
$$

### 4.6.2 中心 → 首格中心

$$
\vec{v}_{\text{origin}} = \frac{1}{2}\mathbf{L}, \quad
\vec{v}_{\text{first}} = \vec{v}_{\text{origin}} - \frac{1}{2}\Delta \cdot \mathbf{1}
$$

### 4.6.3 Position ↔ Index

**位置 → 索引**：

$$
\vec{i}_{\text{vec}} = \frac{\mathbf{p} - \vec{v}_{\text{origin}} - \mathbf{p}_{\text{map}}}{\Delta}
$$

经 buffer 变换 $T_{\text{buf}} = -\mathbf{I}_2$：

$$
\mathbf{i}_{\text{buffer}} = \mathrm{wrap}\!\left(T_{\text{buf}}(\vec{i}_{\text{vec}}) + \mathbf{s}_{\text{start}}\right)
$$

**索引 → 位置**：

$$
\mathbf{p} = \mathbf{p}_{\text{map}} + \vec{v}_{\text{first}} + \Delta \cdot T_{\text{buf}}^{-1}(\mathbf{i}_{\text{unwrapped}})
$$

### 4.6.4 与 Costmap2D 坐标对比

| 属性 | Costmap2D | GridMap |
|------|-----------|---------|
| 原点 | 左下角 $(x_0, y_0)$ | 中心 $\mathbf{p}_{\text{map}}$ |
| 索引方向 | $idx = m_y \cdot N_x + m_x$ | Eigen 列主序 + $T_{\text{buf}} = -I$ |
| 无效值 | `NO_INFORMATION` (255) | `NaN` |

## 4.7 循环 Buffer 与 move()

GridMap 使用**循环 buffer** 实现高效滑动窗口，数据在 map frame 中保持 stationary。

### 4.7.1 move 算法

```cpp
grid_map.move(new_position);
```

步骤：

1. $\Delta \mathbf{p} = \mathbf{p}_{\text{new}} - \mathbf{p}_{\text{map}}$
2. $\Delta \mathbf{i} = \mathrm{round}(\Delta \mathbf{p} / \Delta)$（带符号舍入）
3. $\Delta \mathbf{p}_{\text{aligned}} = \Delta \mathbf{i} \cdot \Delta$
4. 移出 buffer 的行/列 `clearRows` / `clearCols`（设为 `NaN`）
5. 更新 `startIndex_` 和 `position_`

### 4.7.2 与 Costmap Rolling Window 对比

| 维度 | Costmap2D `updateOrigin` | GridMap `move` |
|------|--------------------------|----------------|
| 数据结构 | 线性数组平移 copy | 循环 buffer 索引旋转 |
| 无效区域 | `default_value_` | `NaN` |
| 原点更新 | 左下角平移 | 中心平移 |
| 重叠保留 | 显式 copy 重叠区 | buffer 自然保留 |

## 4.8 插值方法

`atPosition(layer, position, value, method)` 支持四种模式：

### 4.8.1 最近邻（INTER_NEAREST）

$$
f(\mathbf{p}) = f(\mathbf{i}_{\text{nearest}})
$$

### 4.8.2 双线性（INTER_LINEAR）

设查询点落在 unit square，参数 $(s, t) \in [0,1]^2$，四角值 $f_{00}, f_{10}, f_{01}, f_{11}$：

$$
f(s,t) = (1-s)(1-t)f_{00} + s(1-t)f_{10} + (1-s)t f_{01} + st f_{11}
$$

### 4.8.3 Keys 双三次卷积（INTER_CUBIC_CONVOLUTION）

1D 卷积（`convolve1D`）：

$$
y = \frac{1}{2} [1,\; t,\; t^2,\; t^3] \cdot M_{\text{conv}} \cdot \mathbf{f}
$$

Keys 矩阵：

$$
M_{\text{conv}} = \begin{bmatrix}
0 & 2 & 0 & 0 \\
-1 & 0 & 1 & 0 \\
2 & -5 & 4 & -1 \\
-1 & 3 & -3 & 1
\end{bmatrix}
$$

2D 情况：先对行做 1D 卷积，再对列做 1D 卷积（可分离）。

### 4.8.4 标准双三次（INTER_CUBIC）

在 unit square 四角取函数值及偏导，组装 `FunctionValueMatrix`：

$$
\frac{\partial f}{\partial x}\bigg|_{i,j} \approx \frac{f_{i+1,j} - f_{i-1,j}}{2\Delta}, \quad
\frac{\partial f}{\partial y}\bigg|_{i,j} \approx \frac{f_{i,j+1} - f_{i,j-1}}{2\Delta}
$$

$$
\frac{\partial^2 f}{\partial x \partial y}\bigg|_{i,j} \approx \frac{f_{i+1,j+1} - f_{i-1,j+1} - f_{i+1,j-1} + f_{i-1,j-1}}{4\Delta^2}
$$

经双三次基函数求值，精度最高但计算量略大。

### 4.8.5 选型建议

| 场景 | 推荐方法 |
|------|----------|
| 实时控制 / 高频查询 | `INTER_NEAREST` 或 `INTER_LINEAR` |
| 可视化 / 离线分析 | `INTER_CUBIC_CONVOLUTION` |
| 需要导数连续 | `INTER_CUBIC` |

## 4.9 子模块

| 路径 | 功能 |
|------|------|
| `grid_map_core/iterators/` | 全图、线、圆、椭圆、多边形、螺旋、滑动窗口、子图迭代器 |
| `grid_map_core/polygon.*` | 多边形运算 |
| `grid_map_core/GridMapMath.*` | 坐标变换、submap 提取 |
| `grid_map_core/cubic_interpolation.*` | 双三次插值实现 |
| `grid_map_cv/` | OpenCV 图像转换、inpaint 修复 |
| `grid_map_loader/` | 从文件加载 GridMap |
| `grid_map_msgs/` | GridMap 消息序列化 |
| `grid_map_costmap_2d/` | Costmap ↔ GridMap 桥接（**当前未实现**） |

## 4.10 GridMapWrapper

路径：`grid_map_wrapper.hpp` / `grid_map_wrapper.cpp`

- 实现 `MapInterface` 生命周期
- 从 `GridMapOptions` proto 构造 `grid_map::GridMap`
- 默认层：`{"elevation"}`（未配置时）
- `Start()` / `Stop()` 主要为状态管理；**无与 Costmap 自动同步**

### 4.7.1 基本用法

```cpp
auto wrapper = std::make_shared<GridMapWrapper>(grid_opts);
wrapper->Start();

auto& gm = wrapper->getGridMap();
gm.setGeometry(grid_map::Length(4.0, 4.0), 0.1);  // 4m×4m, 10cm
gm.add("elevation");

grid_map::Position pos(0.5, 0.5);
double elev;
gm.atPosition("elevation", pos, elev, grid_map::InterpolationMethods::INTER_LINEAR);
```

## 4.11 与 Costmap2D 的关系

```
┌──────────────────┐          ┌──────────────────┐
│   Costmap2D      │          │    GridMap       │
│   uint8 代价      │   ???    │   float 多层      │
│   规划 / 避障      │ ◄──────► │   地形 / 高程     │
└──────────────────┘          └──────────────────┘
         grid_map_costmap_2d/（已注释，未实现）
```

**潜在桥接方案**（`costmap_2d_converter.hpp` 中已注释的逻辑）：

- 将 `elevation` 或 `traversability` 层线性映射到 $[0, 252]$ 代价值
- 处理 Y 轴反向与原点偏移
- 合并到 Costmap2D 的 obstacle 或独立图层

当前需应用层自行实现互转。

## 4.12 典型应用场景

| 场景 | 使用层 | 说明 |
|------|--------|------|
| 越野/台阶检测 | `elevation` + `slope` | 基于高程差计算坡度 |
| 不确定性融合 | `variance` | 过滤低置信度区域 |
| 语义分割融合 | `semantic` | 多类别标签编码 |
| 点云累积 | `elevation` | 多帧激光/深度相机融合 |
| 可通行性评估 | `traversability` | 映射到规划代价（需桥接） |

## 4.13 相关文档

- [§4.6–§4.8 坐标与插值](04_grid_map.md#46-坐标变换)
- [模块架构设计](01_architecture.md)
- [地图表示综述](05_survey.md)
- [Costmap2D 代价地图](03_costmap2d.md)
