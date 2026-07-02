(map-overview)=
# 1. 模块概览

### 1.1 定位

| 维度 | 说明 |
|------|------|
| 模块职责 | 环境地图的加载、融合、更新与发布 |
| 核心输出 | `OccupancyGrid`（静态）、`Costmap2D`（规划/避障）、`GridMap`（地形/高程） |
| 上游 | SLAM、激光雷达、点云、静态地图文件（YAML+PGM） |
| 下游 | `planning`（全局规划）、`control`（局部避障）、可视化 |
| 对标 | nav2_map_server、nav2_costmap_2d、grid_map（ETH） |

### 1.2 三大子系统

| 子系统 | 类名 | 数据类型 | 典型用途 |
|--------|------|----------|----------|
| 静态地图服务 | `MapServer` | `OccupancyGrid`（-1/0/100） | 加载 SLAM 地图、发布 `/map` |
| 2D 代价地图 | `Costmap2DWrapper` | `uint8` 代价值 0–255 | 全局/局部路径规划、碰撞检测 |
| 2.5D 多层栅格 | `GridMapWrapper` | `float` 多层矩阵 | 地形高程、坡度、多模态感知 |

三者通过统一生命周期接口 `MapInterface`（`Start` / `Stop` / `Pause` / `Resume`）管理；`Costmap2DWrapper` 与 `GridMapWrapper` 各自独立运行，**当前代码库中二者尚未打通桥接**。

### 1.3 源码结构

```
autonomy/map/
├── common/
│   └── map_interface.hpp          # 统一生命周期接口
├── map_server.hpp / .cpp          # 静态 OccupancyGrid 服务
├── map_options.hpp / .cpp         # Lua → Protobuf 配置加载
├── proto/
│   ├── map_options.proto          # MapServer 选项
│   ├── map_2d_option.proto        # Costmap2D 及图层选项
│   └── map_grid_option.proto      # GridMap 选项
├── costmap_2d/                    # 2D 代价地图（Nav2 架构）
│   ├── costmap_2d.hpp             # 核心栅格与坐标变换
│   ├── layered_costmap.*          # 多层聚合
│   ├── costmap_2d_wrapper.*       # 运行时封装
│   ├── layer.hpp / costmap_layer.*# 图层抽象
│   ├── layers/                    # static / obstacle / inflation / voxel ...
│   ├── filters/                   # keepout / speed / binary
│   └── footprint.* / costmap_math.*
├── grid_map/                      # 2.5D 多层栅格（ETH grid_map）
│   ├── grid_map_wrapper.*         # Autonomy 封装
│   ├── grid_map_core/             # GridMap、数学、插值、迭代器
│   ├── grid_map_cv/               # OpenCV 处理
│   └── grid_map_costmap_2d/       # 桥接（当前未实现）
└── utils/                         # PGM 转换、数据加载
```

### 1.4 相关模块

- `autonomy/planning` — 通过 `Costmap2DWrapper` 消费全局代价地图
- `autonomy/transform` — 坐标变换（TF），传感器数据对齐到地图坐标系
- `autonomy/commsgs` — `OccupancyGrid`、`GridMap` 等消息定义
- `config/planner/planner.lua` — 全局 costmap 配置示例

### 1.5 坐标系约定差异（重要）

| 属性 | Costmap2D | GridMap |
|------|-----------|---------|
| 原点 | 地图**左下角** $(x_0, y_0)$ | 地图**中心** $\mathbf{p}_{\text{map}}$ |
| 索引 | $idx = m_y \cdot N_x + m_x$ | Eigen 矩阵 + 循环 buffer 翻转 |
| 数值 | 离散代价值 0–255 | 连续浮点，无效值为 `NaN` |

互转或联合使用时必须显式处理原点与轴向差异，详见 [§3 数学原理](03_math.md)。
