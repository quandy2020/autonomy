(localization-overview)=
# 1. 模块概览

### 1.1 定位

| 维度 | 说明 |
|------|------|
| 模块职责 | 估计机器人在全局/局部坐标系中的位姿；可选同步构建地图 |
| 核心输出 | 位姿 / 里程计、`map`↔`odom`↔`base_link` TF、占用栅格 |
| 上游 | 激光、IMU、里程计 |
| 下游 | `map`、`planning`、`control`、可视化 |
| 对标 | Cartographer、nav2_amcl |

### 1.2 子系统

| 子系统 | 目录/配置 | 传感器 | 典型场景 | 代码状态 |
|--------|-----------|--------|----------|----------|
| **Cartographer** 激光 SLAM | `cartographer/` | 激光 + IMU | 2D/3D 建图与定位 | **已实现**（`localization` 默认） |
| **Lightning** LIO | `lightning/` | 激光 + IMU | 独立 LIO | **已实现** |
| **AMCL** 粒子滤波 | `config/localization/amcl/` | 激光 + 里程计 + 先验地图 | 2D 室内定位 | 配置就绪，C++ 待集成 |

`localization` 二进制：`--localization_mode=cartographer`（默认）或 `lightning`。

### 1.3 Cartographer 双进程架构（可选）

```
  echoes_1 / scan + imu + tf
           │
           ▼
  ┌────────────────────┐
  │  CartographerNode  │──► submap_list, tracked_pose, TF map→odom
  └─────────┬──────────┘
            │ (可选)
            ▼
  ┌─────────────────────────────┐
  │ cartographer_occupancy_grid │──► /map (OccupancyGrid)
  └─────────────────────────────┘
```

单进程模式（`backpack_2d.lua`）将占据栅格发布内嵌于 `CartographerNode`。

### 1.4 源码结构

```
autonomy/localization/
├── localization_main.cpp           # 统一入口：cartographer | lightning
├── localization_server.*           # 后端门面
├── launch/                         # autolink launch
├── conf/{cartographer,lightning}/
├── cartographer/                   # 激光 SLAM
└── lightning/                      # 独立 LIO
```

### 1.5 相关模块

- `autonomy/map` — 静态地图 / MapServer
- `autonomy/transform` — TF 树：`map` → `odom` → `base_link`
- `autonomy/driver` — 传感器数据转发
- `autonomy/visualization` — 位姿 / 里程计可视化话题

### 1.6 坐标系约定

Cartographer / Lightning 遵循 ROS 惯用右手系与 TF 约定。
