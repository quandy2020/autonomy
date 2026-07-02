(localization-overview)=
# 1. 模块概览

### 1.1 定位

| 维度 | 说明 |
|------|------|
| 模块职责 | 估计机器人在全局/局部坐标系中的位姿；可选同步构建稀疏地图 |
| 核心输出 | 相机/机器人位姿 $T_{wc}$、`map`↔`odom`↔`base_link` TF、稀疏 3D 路标地图 |
| 上游 | 相机（单目/双目/RGB-D）、激光（AMCL 规划）、里程计 |
| 下游 | `map`（静态地图）、`planning`（全局规划）、`control`（局部跟踪）、可视化 |
| 对标 | ORB-SLAM3 / stella_vslam（Atlas）、nav2_amcl、Cartographer |

### 1.2 三大子系统（规划 vs 现状）

| 子系统 | 目录/配置 | 传感器 | 典型场景 | 代码状态 |
|--------|-----------|--------|----------|----------|
| **Atlas** 视觉 SLAM | `atlas/` | 相机 | 视觉导航、VSLAM 建图 | **已实现** |
| **AMCL** 粒子滤波 | `config/localization/amcl/` | 激光 + 里程计 + 先验地图 | 2D 室内定位 | 配置就绪，C++ 待集成 |
| **Cartographer** 激光 SLAM | `config/localization/cartographer/` | 激光 + IMU | 2D/3D 激光 SLAM | 配置就绪，C++ 待集成 |

当前 `autonomy/localization` 源码**仅包含 Atlas**；`config/localization/localization.lua` 默认算法为 `amcl`，与代码实现存在差距，集成时需显式选择 Atlas 或等待 AMCL 落地。

### 1.3 Atlas 三线程架构

Atlas 采用经典 **Tracking – Mapping – Global Optimization** 三模块并行：

```
                    ┌─────────────────┐
  图像帧 ──────────►│ tracking_module │──► 实时位姿 T_cw
                    └────────┬────────┘
                             │ 关键帧
                    ┌────────▼────────┐
                    │ mapping_module  │──► 局部地图 + Local BA
                    └────────┬────────┘
                             │ 关键帧 + BoW
                    ┌────────▼──────────────────┐
                    │ global_optimization_module│──► 回环检测 + Loop BA
                    └───────────────────────────┘
```

| 模块 | 线程 | 频率 | 职责 |
|------|------|------|------|
| `tracking_module` | 主线程（调用方） | 传感器帧率 | 跟踪、重定位、关键帧决策 |
| `mapping_module` | 独立 `mapping_thread_` | 关键帧触发 | 三角化、局部 BA、地图清理 |
| `global_optimization_module` | 独立 `global_optimization_thread_` | 关键帧触发 | BoW 回环、Sim3 校正、全局 BA |

### 1.4 源码结构

```
autonomy/localization/
└── atlas/                              # 视觉 SLAM 引擎（stella_vslam 架构）
    ├── system.hpp / .cpp               # 系统入口：startup / feed_* / I/O
    ├── tracking_module.*               # 前端跟踪
    ├── mapping_module.*                # 局部建图
    ├── global_optimization_module.*    # 回环与全局优化
    ├── config.hpp / .cpp               # YAML 配置加载
    ├── type.hpp                        # Eigen 类型别名
    ├── camera/                         # 相机模型（透视/鱼眼/等距/径向分割）
    ├── feature/                        # ORB 特征提取
    ├── data/                           # frame / keyframe / landmark / map_database
    ├── match/                          # 投影/立体/BoW/鲁棒匹配
    ├── solve/                          # PnP / Essential / Homography / Fundamental
    ├── initialize/                     # 单目/双目初始化
    ├── optimize/                       # g2o 位姿优化 / LBA / GBA / Sim3
    ├── module/                         # 子模块：initializer / relocalizer / loop_detector ...
    ├── io/                             # SQLite3 / msgpack 地图持久化
    ├── publish/                        # 地图与帧可视化发布
    └── example/                        # EuRoC / KITTI / TUM 等数据集配置

config/localization/
├── localization.lua                    # 顶层：default_algorithm = "amcl"
├── amcl/amcl.lua                       # AMCL 参数（nav2 对齐）
└── cartographer/*.lua                  # Cartographer 参数模板
```

### 1.5 支持的传感器模式

| 模式 | `setup_type_t` | 输入 | 深度来源 |
|------|----------------|------|----------|
| 单目 | `Monocular` | 灰度/RGB 图像 | 运动恢复结构（SfM） |
| 双目 | `Stereo` | 左右已校正图像 | 视差三角化 |
| RGB-D | `RGBD` | 对齐的 RGB + 深度图 | 深度传感器 |

### 1.6 相关模块

- `autonomy/map` — 消费 Atlas 建图结果或 AMCL 定位后的静态地图
- `autonomy/transform` — TF 树：`map` → `odom` → `base_link` → `camera`
- `autonomy/driver` — 传感器数据转发（`forward_targets = {"localization"}`）
- `autonomy/visualization` — `/localization/pose`、`/localization/odometry` 话题

### 1.7 坐标系约定

| 符号 | 含义 | Atlas 代码 |
|------|------|------------|
| $T_{cw}$ / `pose_cw` | 世界 → 相机（4×4 齐次变换） | 帧/关键帧位姿 |
| $T_{wc}$ | 相机 → 世界 | 取逆得到 |
| $\mathbf{b}$ | 单位化 bearing vector（相机坐标系） | `camera::base::convert_keypoint_to_bearing` |
| $\mathbf{X}_w$ | 路标世界坐标 | `landmark::pos_w_` |

**注意**：Atlas 使用 **右手坐标系**，位姿存储为 $T_{cw}$（将世界点变换到相机系）。与 ROS `geometry_msgs/Pose` 互转时需统一四元数顺序（Eigen `Quaterniond(w,x,y,z)`）。
