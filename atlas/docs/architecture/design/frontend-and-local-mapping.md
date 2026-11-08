# 初始化、里程计与局部建图

所有路径以 C++ 源码根目录 `atlas/atlas/` 为基准；为规划名，不代表已落地。已有同职责文件先复用，不新增重复实现。

| 文件 | 主类型 | 职责 |
|---|---|---|
| `frontend/initialization/state_initializer.hpp/.cpp` | `StateInitializer` | 粗略初值、粗到细 BA 调度和初始化重试 |
| `frontend/initialization/initialization_options.hpp` | `InitializationOptions` | 初始化数值门限和窗口配置 |
| `frontend/initialization/initialization_result.hpp` | `InitializationResult` | 初始状态及明确失败原因 |
| `frontend/initialization/initialization_criteria.hpp/.cpp` | `InitializationCriteria` | 收敛、重力模长、平面法向可观性检查 |
| `frontend/initialization/gravity_alignment.hpp/.cpp` | `GravityAlignment` | 将估计重力对齐世界 z 轴并转换初始状态 |
| `frontend/inertial/inertial_state_filter.hpp/.cpp` | `InertialStateFilter` | 惯性误差状态传播和更新；迁移时复用现有算法实现 |
| `frontend/inertial/inertial_preintegrator.hpp/.cpp` | `InertialPreintegrator` | 相邻状态间预积分；逐步使用强类型 node ID |
| `frontend/odometry/odometry_estimator.hpp/.cpp` | `OdometryEstimator` | 当前状态估计，连接惯性预测与雷达测量更新 |
| `frontend/odometry/scan_matching.hpp/.cpp` | `ScanMatching` | 当前扫描与局部体素平面关联、生成测量 |
| `frontend/odometry/degeneracy_evaluator.hpp/.cpp` | `DegeneracyEvaluator` | 评估独立平面方向约束和退化状态 |
| `frontend/odometry/point_covariance_estimator.hpp/.cpp` | `PointCovarianceEstimator` | 点测量及平面匹配所需的不确定度估计 |
| `frontend/local_mapping/local_trajectory_builder.hpp/.cpp` | `LocalTrajectoryBuilder` | 局部轨迹处理、滑窗更新和边缘化输出 |
| `frontend/local_mapping/optimization/local_bundle_adjustment.hpp/.cpp` | `LocalBundleAdjustment` | 局部状态、惯性预积分和点云几何代价的 BA 问题 |
| `frontend/local_mapping/optimization/point_cloud_factor.hpp/.cpp` | `PointCloudFactor` | 点簇协方差特征值代价与解析导数 |
| `frontend/local_mapping/optimization/inertial_factor.hpp/.cpp` | `InertialFactor` | 窗口内相邻状态的惯性约束 |
| `frontend/local_mapping/local_trajectory_builder_state.hpp` | `LocalTrajectoryBuilderState` | builder 私有窗口状态和工作缓存 |
| `mapping/point_cluster.hpp` | `PointCluster` | 点数、一阶/二阶矩和紧凑坐标变换 |
| `mapping/adaptive_voxel_map.hpp/.cpp` | 目标 `AdaptiveVoxelMap`（未实现） | 目标中的自适应地图边界；当前代码使用 `sliding_voxel_tree.hpp` 与 `voxel_plane_map.*` |

点云降采样属于 `frontend/odometry/` 的算法处理路径；不保留未被 Voxel-SLAM 调用的通用 Cartographer `sensor/internal/voxel_filter`。

`frontend/` 根目录放置阶段编排与轨迹生命周期；初始化、惯性估计、里程计和局部建图分别使用目标树中的职责目录。只有求解器实现细节可在相应模块下继续细分 `internal/` 或 `optimization/`，不额外套一层通用 `frontend/internal/`。若当前 `Initialization`、`LocalMapping` 或 `PointCloudFactor` 已具备相同职责，应先评估保留并改名/扩展，不要并列增加近似 facade。`LocalTrajectoryBuilder` 只有在它确实拥有窗口生命周期和输出边界时才值得独立存在。
