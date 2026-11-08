# 文件与类的一对一设计

本文把目录树中的目标文件落实为类边界。约定适用于 C++17，文件名沿用 Cartographer 风格的短领域名，不在文件名重复模块前缀。类名按领域语义取名，如 `TrajectoryNode`、`PoseGraph`、`PoseConstraint`、`LocalTrajectoryBuilder`；算法类说明处理对象和算法阶段。模块路径已提供上下文时，不在类名重复 `Voxel`、`Backend`、`Frontend` 等前缀。Cartographer 名称仅用于说明领域建模来源；本设计列出的 Atlas 类名均至少由两个完整词组成。这里描述目标职责，不代表类和文件已经实现或迁移。

## 设计约束

1. 一个公开头文件对应一个主要类；同一类的声明和定义分别放在同名 `.hpp`、`.cpp`。纯接口、值对象、配置和结果类型也各占自己的头文件。
2. 一个类只拥有一种变化原因：地图容器不负责序列化，序列化器不选择加载哪些会话，后端调度不实现 HBA 数学，增量位姿图求解实现 不维护 PoseGraph 领域状态。
3. 不为每个小函数机械创建类。无状态的短私有函数留在对应 `.cpp` 匿名命名空间；若需要独立状态、生命周期、可替换契约或独立测试，再提取类。
4. `enum class`、`using` 和该类的私有小型实现细节可与主类同文件；不能把两个独立公共类塞进一个头文件。多个独立配置/结果 struct 分开文件。
5. 不建立只转发调用的 `Manager`、`Helper`、`Engine`、`Worker`。类必须拥有可说明的状态或算法责任。
6. 公共类依赖值对象、const 引用或只读 snapshot；跨模块可变状态只有一个 owner。异步请求和结果携带 session/map/graph version。
7. 所有权优先使用值语义、`std::unique_ptr` 和不可变 `std::shared_ptr<const T>`。禁止裸指针表达所有权；非拥有指针须明确生命周期。
8. GTSAM 必选，第三方类型仅在 `backend/pose_graph/pose_graph_solver.cpp` 出现；PCL 不进入依赖图，所需能力由 Atlas 类提供。

## Cartographer 命名取舍

参考 Cartographer 的 [mapping 目录](https://github.com/cartographer-project/cartographer/tree/master/cartographer/mapping)和领域类名 `TrajectoryNode`、`PoseGraph`、`Submap`、`MapBuilder`：类名从领域对象与稳定职责出发，目录负责补足上下文。这里借鉴这种组织方法，不照搬它的 ROS/protobuf 依赖、`.h` 后缀或 `IMU`、`Node` 等项目惯例缩写。

- 领域实体使用名词：`TrajectoryNode`、`PoseConstraint`、`MappingSession`、`MapSnapshot`。Cartographer 的单词类名不会直接作为 Atlas 类型名。
- 负责明确动作的类使用领域名词加动作角色：`SessionRegistrar`、`PlaneEstimator`、`LoopGeometryVerifier`、`SessionFileReader`。
- 避免含义空泛的 `Data`、`Processor`、`Manager`、`Helper`。状态对象优先叫 `...State`，观测值叫 `...Observation`，地图内部存储叫 `...Storage`。
- 目录已经限定语义时，不把完整目录重复进类名；例如 `triangle/loop_detector.hpp` 中直接使用 `LoopDetector`。
- 不为看起来“一文件一类”而创建无状态 `Logging` 类、单函数 wrapper 或仅转发接口。可以有明确的 namespace 函数 API；拆类以真实职责、状态和生命周期为依据。
- 类型和文件都使用完整、可读的单词；只保留用户认可的行业/算法术语，如 BTC、GTSAM、Eigen、HBA。变量名和成员字段后续逐项遵守 Google C++ 命名规则并写全语义。

## 应用入口

| 文件 | 唯一主类 | 单一职责与边界 |
|---|---|---|
| `mapping_service.hpp`<br>`mapping_service.cpp` | `MappingService` | 应用级生命周期和稳定输入/输出；持有 builder、后台服务与配置，不实现核心算法 |
| `sensor_message_adapter.hpp`<br>`sensor_message_adapter.cpp` | `SensorMessageAdapter` | AutoMsgs 与 Atlas sensor 值类型转换；不拥有消息队列或算法状态 |

## Frontend 前端编排

### 轨迹编排与初始化

| 文件 | 唯一主类 | 单一职责与边界 |
|---|---|---|
| `trajectory_builder.hpp`<br>`trajectory_builder.cpp` | `TrajectoryBuilder` | 有序编排初始化、惯性、里程计和局部建图；不持有 solver 内部状态 |
| `trajectory_builder_output.hpp` | `OdometryOutput`、`ProcessStatus` | 表达一轮前端处理结果和状态；算法入口直接使用 `TrajectoryBuilder` |
| `trajectory_builder_options.hpp` | `TrajectoryBuilderOptions` | builder 构造所需的聚合配置；子模块配置仍由各自 options 类型持有 |
| `trajectory_builder_state.hpp` | `TrajectoryBuilderState` | builder 当前拥有的阶段状态、序号和版本；不作为跨模块消息 |
| `trajectory_lifecycle.hpp` | `TrajectoryLifecycle` | 初始化、跟踪、退化、结束的合法状态迁移；状态枚举可同文件 |
| `state_initializer.hpp`<br>`state_initializer.cpp` | `StateInitializer` | 协调初始化窗口和估计步骤；不重复实现判据或重力旋转 |
| `initialization_options.hpp` | `InitializationOptions` | 初始化数值门限和窗口配置 |
| `initialization_result.hpp` | `InitializationResult` | 初始状态、成功状态及可诊断失败原因 |
| `initialization_criteria.hpp`<br>`initialization_criteria.cpp` | `InitializationCriteria` | 评估初始化是否满足观测性、重力和收敛条件 |
| `gravity_alignment.hpp`<br>`gravity_alignment.cpp` | `GravityAlignment` | 计算重力方向到世界坐标系的旋转 |

### 惯性估计

| 文件 | 唯一主类 | 单一职责与边界 |
|---|---|---|
| `inertial_state_filter.hpp`<br>`inertial_state_filter.cpp` | `InertialStateFilter` | 惯性状态误差传播和测量更新 |
| `inertial_preintegrator.hpp`<br>`inertial_preintegrator.cpp` | `InertialPreintegrator` | 单段 IMU 预积分及其协方差/Jacobian 累积 |

### 里程计

| 文件 | 唯一主类 | 单一职责与边界 |
|---|---|---|
| `odometry_estimator.hpp`<br>`odometry_estimator.cpp` | `OdometryEstimator` | 对单个时间步协调预测与测量更新并产出状态 |
| `scan_matching.hpp`<br>`scan_matching.cpp` | `ScanMatching` | 当前扫描与局部地图关联并计算匹配测量 |
| `degeneracy_evaluator.hpp`<br>`degeneracy_evaluator.cpp` | `DegeneracyEvaluator` | 根据约束方向和信息矩阵评估观测退化程度 |
| `point_covariance_estimator.hpp`<br>`point_covariance_estimator.cpp` | `PointCovarianceEstimator` | 估计点及局部几何测量的不确定度 |

### 局部建图

| 文件 | 唯一主类 | 单一职责与边界 |
|---|---|---|
| `local_trajectory_builder.hpp`<br>`local_trajectory_builder.cpp` | `LocalTrajectoryBuilder` | 管理局部窗口、关键帧生成和边缘化边界 |
| `local_trajectory_builder_state.hpp` | `LocalTrajectoryBuilderState` | 局部 builder 独占的窗口状态与临时缓存 |
| `local_bundle_adjustment.hpp`<br>`local_bundle_adjustment.cpp` | `LocalBundleAdjustment` | 维护局部 BA 的变量、因子和求解输入输出 |
| `point_cloud_factor.hpp`<br>`point_cloud_factor.cpp` | `PointCloudFactor` | 点簇/平面几何残差及导数 |
| `inertial_factor.hpp`<br>`inertial_factor.cpp` | `InertialFactor` | 相邻状态间 IMU 约束残差 |

## Mapping 地图模块

### 体素地图

| 文件 | 唯一主类 | 单一职责与边界 |
|---|---|---|
| `cell_index.hpp` | `CellIndex` | 体素整数坐标和层级身份；hash functor 同文件，不另算公共领域类 |
| `point_cluster.hpp` | `PointCluster` | 点数、一阶/二阶矩及统计合并/刚体变换 |
| `fitted_plane.hpp` | `FittedPlane` | 已拟合平面的中心、法向、特征值和有效性 |
| `adaptive_octree.hpp`<br>`adaptive_octree.cpp` | `AdaptiveOctree` | 父子节点、分裂/合并和层级遍历 |
| `adaptive_voxel_map.hpp`<br>`adaptive_voxel_map.cpp` | `AdaptiveVoxelMap`（目标设计，未实现） | 目标体素地图插入、删除、查询和快照发布；当前代码没有该类 |
| `sliding_voxel_map.hpp`<br>`sliding_voxel_map.cpp` | `SlidingVoxelMap` | 有限局部空间窗口的推进及越界体素淘汰 |
| `map_snapshot.hpp` | `MapSnapshot` | 带版本、坐标系和分辨率的不可变地图视图 |
| `keyframe_map_observation.hpp` | `KeyframeMapObservation` | 关键帧关联的地图观测值，不拥有地图索引 |
| `hierarchical_bundle_adjustment.hpp`<br>`hierarchical_bundle_adjustment.cpp` | `HierarchicalBundleAdjustment` | 对固定输入运行 HBA 几何优化并返回 pose updates；不排程、不改 PoseGraph |
| `sliding_voxel_tree.hpp` | `SlidingVoxelTree` | `SlidingVoxelMap` 私有的局部八叉树存储/重切实现 |
| `bundle_adjustment_octree.hpp` | `BundleAdjustmentOctree` | HBA 所需分层点簇和邻域统计 |
| `plane_estimator.hpp`<br>`plane_estimator.cpp` | `PlaneEstimator` | 点簇协方差特征分解和平面有效性估计 |
| `map_storage.hpp` | `MapStorage`（可选） | 仅当索引、缓冲和版本状态值得独立隐藏/测试时提取；简单成员直接归属地图容器 |

### 会话文件

| 文件 | 唯一主类 | 单一职责与边界 |
|---|---|---|
| `session_file_header.hpp` | `SessionFileHeader` | 文件版本、会话身份、坐标系、能力和条目元数据 |
| `session_file_version.hpp` | `SessionFileVersion` | 文件版本比较和兼容范围；作为独立值类单独定义 |
| `session_file_reader.hpp`<br>`session_file_reader.cpp` | `SessionFileReader` | 校验并读取一个会话文件，返回完整临时结果 |
| `session_read_result.hpp` | `SessionReadResult` | 一次读取的完整会话值或结构化失败原因 |
| `session_file_writer.hpp`<br>`session_file_writer.cpp` | `SessionFileWriter` | 编码并原子发布一个会话文件 |
| `session_write_result.hpp` | `SessionWriteResult` | 一次写入的状态、目标位置和结构化失败原因 |
| `trajectory_serializer.hpp`<br>`trajectory_serializer.cpp` | `TrajectorySerializer` | 轨迹状态记录与编码转换 |
| `keyframe_serializer.hpp`<br>`keyframe_serializer.cpp` | `KeyframeSerializer` | 关键帧元数据和外部观测引用编解码 |
| `voxel_map_serializer.hpp`<br>`voxel_map_serializer.cpp` | `VoxelMapSerializer` | 体素键、点簇统计、平面和地图版本编解码 |
| `pose_constraint_serializer.hpp`<br>`pose_constraint_serializer.cpp` | `PoseConstraintSerializer` | 位姿图约束及其方向、不确定度和来源编解码 |
| `binary_reader.hpp`<br>`binary_reader.cpp` | `BinaryReader` | 带边界和字节序检查的底层字节读取 |
| `binary_writer.hpp`<br>`binary_writer.cpp` | `BinaryWriter` | 显式字节序和宽度的底层字节写入 |
| `session_file_checksum.hpp`<br>`session_file_checksum.cpp` | `SessionFileChecksum` | 会话文件完整性计算/校验 |
| `session_data.hpp` | `SessionData` | 当前兼容格式下用于保存的拥有型会话值 |
| `session_writer.hpp`<br>`session_writer.cpp` | `SessionWriter` | 当前兼容格式下写入姿态文本和关键帧点云目录；后续格式升级后由会话文件 writer 替代 |
| `session_reader.hpp`<br>`session_reader.cpp` | `SessionReader` | 按姿态列表和关键帧序号流式读取当前会话目录格式；不构造关键帧或描述子索引 |
| `atomic_file_writer.hpp`<br>`atomic_file_writer.cpp` | `AtomicFileWriter` | 临时文件、flush/close 和原子 rename 生命周期 |
| `legacy_session_importer.hpp`<br>`legacy_session_importer.cpp` | `LegacySessionImporter` | 只读导入原始 Voxel-SLAM 文件，不写旧格式 |

### 地图可视化

| 文件 | 唯一主类 | 单一职责与边界 |
|---|---|---|
| `map_view.hpp` | `MapView` | 地图展示所需的紧凑只读值 |
| `trajectory_view.hpp` | `TrajectoryView` | 轨迹展示所需的只读路径值 |
| `map_view_builder.hpp`<br>`map_view_builder.cpp` | `MapViewBuilder` | 地图快照到展示值的投影和降采样 |
| `trajectory_view_builder.hpp`<br>`trajectory_view_builder.cpp` | `TrajectoryViewBuilder` | 位姿快照到路径和关键帧视图的投影 |
| `visualization_options.hpp` | `VisualizationOptions` | 展示投影范围、字段和输出上限 |

`mapping/visualization/` 是按消费需求启用的模块；只有应用工具时，保留投影逻辑在 app 层，不创建空类。序列化里纯字节函数可以是 `BinaryReader`/`BinaryWriter` 的私有方法，不额外创建一类仅有一个函数的 wrapper。

## Backend 后端模块

### 回环检测

| 文件 | 唯一主类 | 单一职责与边界 |
|---|---|---|
| `loop_candidate.hpp` | `LoopCandidate` | 一条待验证候选的身份、分数和初始相对变换 |
| `loop_verification_result.hpp` | `LoopVerificationResult` | 几何验证结果、拒绝原因和诊断指标 |
| `loop_closure_options.hpp` | `LoopClosureOptions` | 候选门控与验证阈值配置 |
| `triangle_descriptor.hpp` | `TriangleDescriptor` | BTC 描述子值及来源身份 |
| `descriptor_extractor.hpp`<br>`descriptor_extractor.cpp` | `DescriptorExtractor`（可选） | 只有描述子提取有独立复用、状态或测试边界时才拆分；否则作为 detector 私有实现 |
| `triangle_loop_detector.hpp`<br>`triangle_loop_detector.cpp` | `TriangleLoopDetector` | 基于 BTC 三角形描述子的回环候选检测 |
| `descriptor_index.hpp`<br>`descriptor_index.cpp` | `DescriptorIndex`（可选） | 仅当索引有独立加载、持久化或并发查询生命周期时拆分 |
| `candidate_selector.hpp`<br>`candidate_selector.cpp` | `CandidateSelector`（可选） | 只有候选选择有独立复用、策略或测试边界时才拆分 |
| `match_verifier.hpp`<br>`match_verifier.cpp` | `MatchVerifier`（可选） | 只有匹配验证有独立复用、状态或测试边界时才拆分 |
| `loop_geometry_verifier.hpp`<br>`loop_geometry_verifier.cpp` | `LoopGeometryVerifier` | 点云几何验证、退化拒绝和测量不确定度估计 |
| `loop_candidate_filter.hpp`<br>`loop_candidate_filter.cpp` | `LoopCandidateFilter`（可选） | 仅当门控策略独立配置或复用时提取；简单门控留在 detector 内 |

### 位姿图

| 文件 | 唯一主类 | 单一职责与边界 |
|---|---|---|
| `pose_constraint.hpp`<br>`pose_constraint.cpp` | `PoseConstraint` | 固定方向的相对位姿测量及数值/不确定度不变式；不检查图端点 |
| `pose_graph.hpp`<br>`pose_graph.cpp` | `PoseGraph` | 唯一拥有图节点、约束和版本；通过 `ApplyOptimizationResult` 校验版本并提交位姿更新 |
| `pose_graph_options.hpp` | `PoseGraphOptions` | 图层策略和门限配置 |
| `pose_graph_result.hpp` | `PoseGraphResult` | 带源图版本的优化更新及诊断 |
| `pose_graph_storage.hpp` | `PoseGraphStorage` | PoseGraph 私有节点/边索引和版本存储 |
| `constraint_validation.hpp`<br>`constraint_validation.cpp` | `ConstraintValidation` | 在图上下文验证端点存在性、重复边和图级约束策略；数值不变式由 `PoseConstraint` 保证 |
| `trajectory_connectivity_state.hpp`<br>`trajectory_connectivity_state.cpp` | `TrajectoryConnectivityState` | 会话连通分量及对齐可达状态 |
| `pose_graph_problem.hpp`<br>`pose_graph_problem.cpp` | `PoseGraphProblem` | 领域图快照到公共 FactorGraph 的映射 |
| `pose_graph_optimizer.hpp`<br>`pose_graph_optimizer.cpp` | `PoseGraphOptimizer` | 调用 GTSAM 增量位姿图求解实现 并映射 Atlas 自有 结果 |

### 全局建图

| 文件 | 唯一主类 | 单一职责与边界 |
|---|---|---|
| `global_mapping.hpp`<br>`global_mapping.cpp` | `GlobalMapping` | 历史恢复、会话收尾和全局优化任务的唯一协调 owner |
| `global_mapping_options.hpp` | `GlobalMappingOptions` | 会话恢复和全局任务策略配置 |
| `global_mapping_request.hpp` | `GlobalMappingRequest` | 含版本与不可变输入快照的一次全局任务请求 |
| `global_mapping_result.hpp` | `GlobalMappingResult` | 任务状态、更新、源版本和诊断 |
| `previous_session_loader.hpp`<br>`previous_session_loader.cpp` | `PreviousSessionLoader` | 发现、读取、验证和原子发布历史会话 |
| `mapping_session.hpp` | `MappingSession` | 会话身份和生命周期状态；文件版本、坐标系等持久化信息归 `SessionFileHeader` |
| `mapping_job.hpp`<br>`mapping_job.cpp` | `MappingJob` | 单次任务执行状态和取消点；不创建自己的线程池 |
| `bundle_adjustment_request.hpp` | `BundleAdjustmentRequest` | 单次 HBA 的窗口/块只读快照、约束和输入版本 |
| `bundle_adjustment_result.hpp` | `BundleAdjustmentResult` | HBA 位姿更新、状态、源版本和分层统计；由 `GlobalMapping` 汇总为全局任务结果 |
| `session_registrar.hpp`<br>`session_registrar.cpp` | `SessionRegistrar` | 计算并验证新旧会话对齐测量 |

## Optimization 与自研点云能力

| 文件 | 唯一主类 | 单一职责与边界 |
|---|---|---|
| `pose_node.hpp` | `PoseNode` | 位姿初值和固定状态 |
| `prior_factor.hpp` | `PriorFactor` | 单节点位姿先验和权重 |
| `between_factor.hpp` | `BetweenFactor` | 两节点相对位姿测量和信息权重 |
| `factor_graph.hpp/.cpp` | `FactorGraph` | 聚合位姿节点、先验因子和相对因子，并校验输入图 |
| `pose_graph_solver.hpp/.cpp` | `PoseGraphSolver` | 持有 ISAM2 状态并执行追加式增量更新；同时定义求解选项、状态和结果 |
| `nearest_neighbor_index.hpp`<br>`nearest_neighbor_index.cpp` | `NearestNeighborIndex` | Atlas 自有只读近邻索引；内部可用 k-d tree 实现 |
| `sensor_point_cloud.hpp` | `SensorPointCloud` | 拥有型、传输无关的点云容器和值语义 |
| `point_with_intensity.hpp` | `PointWithIntensity` | 位置和强度字段及单位约定；不同点型分别定义独立头文件 |

点云降采样由前端里程计实际使用的预处理实现承担；不保留未进入 Voxel-SLAM 数据流的通用传感器 voxel filter。

## 线程池与日志/配置边界

| 文件 | 唯一主类 | 单一职责与边界 |
|---|---|---|
| `thread_pool.hpp`<br>`thread_pool.cpp` | `ThreadPool` | 通用任务执行、队列上限和安全停止；不包含 SLAM 任务策略 |
| `thread_pool_options.hpp` | `ThreadPoolOptions` | worker 数、队列上限和关闭策略 |
| `logging.hpp` | `atlas::logging` 函数接口 | Atlas 日志 facade；不制造无状态 `Logging` 类，第三方 glog 宏不进入领域公共头 |
| `configuration_view.hpp` | `ConfigurationView` | 与具体 parser 无关的只读配置查询契约 |
| `opencv_configuration_reader.hpp`<br>`opencv_configuration_reader.cpp` | `OpenCvConfigurationReader` | 可选 OpenCV 配置格式 adapter，不进入核心依赖；OpenCV 的大小写按官方产品名书写 |

## 实施顺序

先选一个垂直切片落实：`PoseGraph`/GTSAM adapter，或 `mapping` 的快照边界。每个切片先确定唯一状态 owner 和值类型，再拆头/源文件，最后更新 CMake target。不要一次性为整张目录树制造只有空类和转发函数的文件。

具体算法输入、输出和迁移证据继续由[前端设计](frontend-and-local-mapping.md)、[位姿图设计](backend/pose-graph.md)、[回环设计](backend/loop-closure.md)、[全局建图设计](backend/global-mapping.md)、[Mapping 子模块设计](mapping-and-data.md)及[依赖方案](third-party-dependencies.md)分别维护。
