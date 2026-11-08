# Voxel-SLAM 移植目录树

本文以原始 Voxel-SLAM 源码和 Atlas 已落地文件为依据，设计后续目录组织。Cartographer 只提供工程组织参考：领域名词命名、公开模型与 `internal/` 实现分离、按职责组织文件；算法模块和数据流仍由 Voxel-SLAM 决定。

## 设计边界

- 复刻目标是原算法链：LiDAR/IMU 同步、初始化、误差状态 EKF 里程计、局部滑窗 LiDAR-IMU BA、自适应体素地图、BTC 回环、增量 PGO、多会话地图、全局 HBA。
- 算法库不依赖 ROS。当前 Atlas 不包含 ROS bag、PointCloud2、MCAP 输入工具；如需接入，应放在独立可选适配目标。AutoMsgs 转换位于 `sensor/msg_conversion.*` 和 `atlas_automsgs` 边界，不把 protobuf/传输类型带入算法模型。
- 保留 Atlas 当前工程根、CMake target 和主要目录；这是目标职责树，不要求一次性移动文件。迁移按模块逐步完成，保证每阶段构建边界清晰。
- 不为了对齐 Cartographer 而强制新增通用 `TrajectoryBuilder`、`LocalSubmap` 或地图层级。只有当 Voxel-SLAM 算法确实需要稳定对象、生命周期或复用时，才引入对应领域类型。
- 最低 C++17；`frontend` 是低延迟路径，`backend` 消费不可变快照并异步优化，二者通过明确定义的数据契约交接。

## 自顶而下与自下而上的设计闭环

工程架构同时采用两种设计方向。它们不是两份互相竞争的目录方案，而是在每个模块边界处相互校验：

### 自顶而下：由算法目标推导模块

先从完整 Voxel-SLAM 工作链定义系统行为，再逐层确定谁拥有状态、谁消费数据、谁负责并发和失败恢复：

```text
建图目标与运行模式
  -> 单会话/多会话 SLAM 数据流程
  -> 前端实时估计与后端全局优化
  -> 初始化、惯性估计、里程计、局部 BA、回环、PGO、HBA
  -> 类职责、输入输出契约、线程/快照边界
  -> 源文件、目录和 CMake target
```

自顶而下需要回答：模块的唯一职责是什么；算法状态归谁所有；输入输出是否稳定且可测；前端是否可能被后台阻塞；会话完成、失败和重启时状态如何迁移。只有这些问题明确后，才新增目录或类。这样可避免把所有逻辑留在 `Odometry`，也避免按论文小节制造没有状态所有权的 facade。

### 自下而上：由现有算法与工程约束校验

从原始 `voxelslam.cpp`、算法头文件和 Atlas 已有代码逐项核对数据结构、数学调用、第三方依赖、线程行为和实时开销，再反馈修订顶层模块设计：

```text
原始算法实现与 Atlas 当前调用链
  -> 状态/缓存/队列/线程及对象生命周期
  -> 每个函数的实际读写与依赖方向
  -> 复用、拆分、合并或保留兼容入口
  -> 对照顶层模块契约与性能预算
  -> 修订接口、类所有权、目录或迁移顺序
```

自下而上需要回答：原算法依赖哪些中间量且是否必须共同更新；哪些数据可变、哪些可安全共享；现有算法是否确实产生 `TrajectoryNode` 或 `LocalSubmap` 语义；后台任务能否使用只读快照；C++17、GTSAM 和 AutoMsgs 的依赖边界是否满足 Atlas 构建约束。若源码证据和概念模型冲突，以保留算法行为为先，再调整抽象和命名。

### 两个方向的汇合规则

| 设计问题 | 自顶而下产物 | 自下而上证据 | 汇合后的决定 |
|---|---|---|---|
| 前后端边界 | 前端必须低延迟，后端可异步 | 检查现有线程循环、缓冲区和地图锁 | 选定快照内容、队列容量、结果提交点 |
| 关键帧/图节点 | 后端需要稳定图变量和约束端点 | 检查原始 `ScanPose`、Keyframe 与 PGO ID | 保留已有语义或建立显式转换，不强行改名成通用 Node |
| 地图块/子地图 | 全局 HBA 需要可处理的空间块 | 检查重叠窗口、体素层级和局部 BA 成员关系 | 只有成员和局部坐标语义吻合时才引入 `LocalSubmap` |
| 模块目录 | 目录应体现稳定责任 | 检查现存文件、实际调用和 CMake 源列表 | 复用已有 `frontend/`、`backend/`、`map/` 等边界，去掉无信息重复层 |
| 线程池任务 | 长任务不可阻塞前端，停止时可控 | 检查 HBA/PGO 并行粒度、临时分配和结果合并 | 线程池只调度适合并行的任务，算法内部同步保持显式 |

因此架构不是自顶而下的一次性蓝图，也不是把现有文件原样搬进新目录。每次迁移都按“顶层契约提出边界 -> 源码证据验证边界 -> 实现/基准暴露问题 -> 回写架构和目录设计”迭代；目录树必须同时通过算法完整性检查和工程依赖检查。

## 目标目录树

```text
atlas/                                  # Atlas 工程根
├── app/                                # 应用入口与外部数据适配
│   ├── CMakeLists.txt                  # 应用库、回放入口和应用测试
│   ├── replay_main.cpp                 # Voxel-SLAM 离线回放入口
│   ├── file_replay_source.hpp/.cpp     # 文本 IMU 与点云数据集读取
│   ├── voxel_slam_system.hpp/.cpp      # 应用级算法生命周期
│   ├── voxel_slam_message_adapter.hpp/.cpp # AutoMsgs 与 Atlas 类型适配
│   ├── slam_system_config.hpp/.cpp    # 应用参数读取、校验与保存
│   └── tools/
│       ├── convert/                     # ROS bag、MCAP 等离线转换
│       ├── eval/
│       └── visualize/
├── config/
│   └── voxel_slam_default.yaml          # 系统唯一 YAML 配置，按模块分区
├── docs/                                # 架构、算法、迁移和使用文档
├── atlas/                               # C++17 核心源码
│   ├── common/                          # 跨模块基础设施
│   │   ├── thread_pool.hpp/.cpp
│   │   ├── thread_safe_queue.hpp
│   │   ├── time.hpp
│   │   └── transform/
│   ├── sensor/                          # 与传输无关的输入值类型和同步
│   │   ├── inertial_data.hpp/.cpp
│   │   ├── point_cloud.hpp               # 拥有型传输无关点云
│   │   ├── sensor_sync.hpp/.cpp
│   │   └── msg_conversion.hpp/.cpp       # 可选 AutoMsgs 适配边界
│   ├── frontend/                        # 前端实时估计与局部状态
│   │   ├── initialization/              # 重力对齐、初始化窗口与判据
│   │   ├── inertial/                     # 惯性状态、滤波与预积分
│   │   ├── odometry/                    # scan-to-map、点协方差、退化检查
│   │   ├── local_mapping/               # 滑窗、LiDAR-IMU BA、边缘化
│   │   ├── trajectory_builder.hpp/.cpp  # 前端阶段编排与轨迹生命周期
│   │   ├── trajectory_builder_state.hpp # 前端编排当前状态和序号
│   │   └── trajectory_lifecycle.hpp     # 初始化、跟踪、退化、结束状态
│   ├── map/                             # Voxel-SLAM 几何地图和地图 IO
│   │   ├── voxel/                       # 自适应体素、八叉树、点簇、HBA 算法
│   │   ├── serialization/               # 地图/会话序列化格式
│   │   └── visual/                      # 地图可视化表示
│   └── backend/                         # 延迟容忍的全局估计
│       ├── loop_closure/                # BTC 检索、候选管理与几何验证
│       │   └── triangle/                # 三角形描述子构造、索引与匹配
│       ├── pose_graph/                  # 图状态、因子、ISAM2 与优化提交
│       │   ├── pose_node.hpp
│       │   ├── prior_factor.hpp
│       │   ├── between_factor.hpp
│       │   ├── factor_graph.hpp/.cpp
│       │   ├── pose_graph_solver.hpp/.cpp
│       │   ├── loop_factor.hpp/.cpp
│       │   └── loop_pose_graph.hpp/.cpp
│       └── global_mapping/              # 多会话、PGO、全局任务生命周期
│           └── internal/                # 快照、任务数据和 HBA 编排
└── tests/ 或模块内 *_test.cpp           # 遵循当前 CMake/gtest 组织
```

`tests/` 仅当仓库统一测试策略要求集中放置时采用；当前 Atlas 的模块内测试文件继续就地组织，不为目录美观批量搬迁。

## 原始源码到 Atlas 的迁移映射

| 原始 Voxel-SLAM 源码/职责 | Atlas 目标位置 | 迁移说明 |
|---|---|---|
| `VoxelSLAM/src/voxelslam.cpp` 中 ROS 回调、主循环、状态机 | `app/voxel_slam_message_adapter.*`、`app/file_replay_source.*`、`app/replay_main.cpp`、`frontend/trajectory_builder.*` | 拆分消息转换、离线读取和算法协调；核心只接收 `sensor` 数据 |
| `voxelslam.cpp` 中初始化实现 | `frontend/initialization/` | 保留粗到细初始化、重力估计与可观性/收敛判定 |
| `ekf_imu.hpp` | `frontend/inertial/inertial_state_filter.*` | 惯性误差状态传播和测量更新 |
| `preintegration.hpp` | `frontend/inertial/inertial_preintegrator.*` | 局部 BA/图约束所需预积分与偏置处理 |
| `voxelslam.cpp` 中 LIO 与 scan-to-map | `frontend/odometry/` | 高频前端路径，局部地图只读访问，避免等待后台图优化 |
| `voxelslam.cpp` 中局部 BA、滑窗和边缘化 | `frontend/local_mapping/` | 保留点簇/IMU 因子、窗口管理与关键帧输出；不把窗口算法放入全局图优化 |
| `voxel_map.hpp`、`feature_point.hpp` | `map/` | 自适应体素、层级结构、平面和点簇数据 |
| `BTC.h/.cpp` | `backend/loop_closure/triangle/` | BTC 描述子和候选检索；候选验证单独维护状态与失败原因 |
| `loop_refine.hpp`、回环 ICP/平面验证 | `backend/loop_closure/` | 以验证通过的相对位姿/协方差作为约束输出 |
| `voxelslam.cpp` 中 GTSAM ISAM2/PGO | `backend/pose_graph/` | 图状态、约束和 ISAM2 增量求解均归后端位姿图模块 |
| 多会话图连接与历史地图 | `backend/global_mapping/`、`map/serialization/` | session 对齐、历史图导入和持久化生命周期分开 |
| bottom-up HBA 与 top-down 全局校正 | `map/` 算法 + `backend/global_mapping/` 调度 | HBA 几何算法保留在 map/optimization；任务快照、线程与提交由后台模块拥有 |
| ROS launch、参数和 PointCloud2 插件 | 外部适配 target | ROS 兼容只在独立边缘适配中提供，不进入核心依赖 |

该映射明确了“复刻算法”和“重构工程”的对应关系：核心算法不能因拆目录而丢失；原实现耦合在主文件中的阶段，需要逐项迁移并用输入/输出契约连接。

逐项迁移还需双向追踪：每个目标模块都要能追溯到原始算法入口/数据结构，每个原始算法阶段也都要在目标模块或明确的兼容策略中有归宿。没有来源的目标类暂不创建；没有目标落点的原算法能力不得在重构中遗漏。

## 与 Cartographer 的对应关系及取舍

| Cartographer 工程经验 | Atlas/Voxel-SLAM 的采用方式 | 不照搬的部分 |
|---|---|---|
| 领域对象有稳定命名，接口与实现边界清楚 | 使用 `pose_graph`、`trajectory_node`（若确需节点模型）、`constraint` 等具体名词；实现细节放 `internal/` | 不为所有组件建立抽象基类 |
| 前端构造器与全局图优化分层 | `frontend/local_mapping` 负责局部估计；`backend` 负责 BTC、PGO、多会话和全局处理 | 不把 Voxel-SLAM 的初始化、滑窗和 HBA 塞入 Cartographer 的 2D/3D 类树 |
| 后台任务与实时路径隔离 | `common/thread_pool` + 有界任务队列；后台仅消费 const 快照，提交时校验 map/session version | 不在每帧路径创建线程；不让 worker 持有前端地图锁 |
| 小而清楚的源文件布局 | 按算法领域拆分，文件名从目录获得上下文 | 不把原本紧密的数学实现拆成大量转发 facade |

### 是否采用轨迹节点和局部子地图

Voxel-SLAM 原始代码围绕 session、scan pose、keyframe、voxel map 和重叠关键帧组组织。后续可以把稳定身份和不可变观测提炼为 `MappingTrajectory`、`TrajectoryNode` 等对象，但必须满足至少一个条件：需要跨会话唯一身份、长期索引、图优化引用，或后台快照共享。`LocalSubmap` 只有在它确实代表固定成员集合和局部坐标系、并参与优化约束时才建立；若当前的 HBA map block 不是这种语义，就继续使用 `MapBlock`/既有领域名词，不强行改称局部子地图。

## 并发和模块依赖

```text
sensor adapter -> sensor sync -> frontend pipeline -> local mapping
                                      |                    |
                                      |                    +-> immutable keyframe/map snapshot
                                      |                              |
                                      +-> odometry output            v
                                                   backend task queue -> BTC / PGO / HBA
                                                                           |
                                  frontend map correction <- versioned result commit
```

- 前端模块不依赖 backend 实现，只发布不可变关键帧/局部地图输入。
- `loop_closure` 生成已经验证的约束，不直接修改地图；`pose_graph` 负责约束和位姿状态；`global_mapping` 负责多会话/HBA 作业生命周期。
- `backend/pose_graph/` 平铺提供 Atlas 位姿图模型和 GTSAM ISAM2 增量求解器；各因子类型分别定义，其他模块通过求解接口取得优化结果。
- 后台队列需有容量和关闭语义；优化结果携带来源 session/map version，过期结果只能被记录和丢弃，不能覆盖新地图状态。
- `sensor` 中使用 Atlas 自有 C++ 值类型。AutoMsgs 仅通过可选转换 target 接入；ROS 适配不进入 `atlas_voxel`。

## 命名约定

- 遵循 Cartographer 的短领域文件名：`pose_graph.hpp`、`pose_constraint.hpp`、`inertial_preintegrator.hpp`、`triangle_descriptor.hpp`。目录已经说明 `backend` 或 `frontend` 时不再加重复前缀。
- 实现文件与声明文件同名；接口后缀 `_interface` 只用于确实需要替换/隔离的契约；`internal/` 用于非 API 实现。
- 文件、目录和新增类型优先使用完整领域单词，例如 `inertial`、`point_cloud`、`trajectory`；仅在论文/算法专名中保留 `BTC`、`PGO`、`HBA` 等约定写法，不使用 `imu` 作为目录名或 `traj`、`kf`、`gba` 等私有缩写。
- 类名表达真实领域状态或算法职责；不新增泛化的 `Manager`、`Helper`、`Engine` 包装类。

## 当前 Atlas 文件放置提醒

以下是整理时应逐步收敛的边界，不是要求立即改名：

1. 活动回环/PGO 图已迁入 `atlas/backend/pose_graph/loop_pose_graph.*`；无产品调用的同步 `PoseGraph`/`BackendRunner` wrapper 已删除。
2. `LoopPoseGraph` 已有版本化快照和过期结果拒绝提交 API；仍需把 `global_mapping/` 后台任务改为 snapshot-solve-commit 流程，并让更新携带 session/map 来源版本。
3. `frontend/trajectory_builder.*` 已承接原主流程入口；后续继续拆出会话生命周期、前端数据和后台提交，同时保持 Voxel-SLAM 算法顺序。
4. `map/hierarchical_bundle_adjustment.*` 继续拥有 HBA 数学实现；线程管理不应继续堆在地图类或 `Odometry` 中。
5. 保持当前 CMake `atlas_voxel` 与可选 `atlas_automsgs` 边界；目录整理不能让核心目标链接 ROS 或 protobuf。

当前 include 方向及具体证据见[模块依赖审计](dependency-audit.md)。地图拥有 `PointCloudFactor`、`PointCloudFactorOptimizer` 和点云因子数据契约，已不再依赖前端 BA 因子头文件；地图对完整前端惯性状态及其协方差传播函数的依赖仍是后续边界迁移项；点观测和平面值类型已归 map 所有，不能因目标目录树已设计完成就视为已解耦。

## 相关模块清单

- [应用与流水线编排](application-and-pipeline.md)
- [Trajectory、Node 与地图模型](trajectory-and-map.md)
- [初始化、里程计与局部建图](frontend-and-local-mapping.md)
- [回环、位姿图与全局建图](backend-and-global-mapping.md)
- [运行时、持久化与测试](runtime-and-storage.md)
