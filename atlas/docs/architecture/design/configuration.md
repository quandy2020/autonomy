# YAML 配置组织与系统契约

本文定义 YAML 与 Atlas 应用配置、Voxel-SLAM 参数之间的对应关系。配置项必须由真实模块消费，并能沿加载、校验、装配链路追溯；示例不代表实现已支持，现行状态见下文。

## 设计约束

- 全系统使用唯一 YAML，参数在文件内按模块职责分区；不为类、模块或场景复制配置文件。
- YAML 只负责应用边界的输入。`app` 层解析并校验，再显式转换为各模块的强类型 options；算法模块不直接读取 YAML。
- 每个参数只有一个权威归属。前端点云降采样属于 `frontend`，地图体素分辨率属于 `mapping`；即使数值相同，也不能共用含义不明的 `voxel_size`。
- 配置字段须与实现参数一一对应。未实现、未接线或语义未查明的参数不进入正式示例。
- 文件名、字段名使用 `lower_snake_case`；物理量字段写明单位或量纲，保留 GTSAM、LiDAR、IMU、BTC 等公认领域缩写。
- 配置分区按数据流和依赖顺序排列：应用/输入 → 前端估计 → 地图 → 优化能力 → 后端全局处理 → 持久化/运行时。初始化是前端阶段，求解器是后端使用的能力；排序不改变二者的模块所有权。

## 文件组织

整个系统只保留一个 YAML：

```text
config/
└── voxel_slam_default.yaml
```

采用 Cartographer Lua 配置的模块分区思想，但不照搬它的多文件 `include`。同一个 YAML 文档内按系统模块嵌套分区，应用从该文件一次性解析出强类型 options。这样既保留单一配置入口，也让参数归属清楚；不维护模块 YAML、profile、继承或覆盖文件。不同数据集的路径通过现有命令行输入提供；算法参数变体仍使用同一文件，避免产生多份相互漂移的配置。

仓库原有 `voxel_slam_frontend_only.yaml`、`voxel_slam_icp_lm.yaml`、`voxel_slam_multisession.yaml` 是空文件，不构成有效配置；现已移除，参数统一由 `config/voxel_slam_default.yaml` 承载。

## 当前加载链路与能力边界

当前链路为：

```text
YAML → app::LoadSlamSystemConfig → app::SlamSystemConfig
    → app::VoxelSlamSystem → frontend::TrajectoryBuilderOptions → frontend::TrajectoryBuilder
```

`LoadSlamSystemConfig` 是手写的 YAML 子集解析器，不是通用 YAML 库。它支持标量和限定的缩进映射；未知字段或无效值会拒绝加载。应用目前只实现 Voxel-SLAM 流水线，旧的 `components` 注册表已移除。配置路径、数据集输入及输出路径由 `app` 层处理。目标字段必须先接入此加载链、强类型配置、校验与消费点，再写入唯一配置文件。

`SlamSystemConfig` 将初始化与局部建图参数保存在 `initialization`、`local_mapping` 中；Voxel-SLAM 专属选项再按 frontend、mapping、optimization、loop closure、global mapping 和 storage 分组。`VoxelSlamSystem` 在应用边界将这些选项一次性转换为算法入口 `frontend::TrajectoryBuilderOptions`。目标 C++ 名称空间为 `atlas::frontend`。

## 当前参数归属

以下映射以现有代码为准。YAML 左列是当前加载器识别的字段，C++ 右列是其最终落点。

| YAML 字段 | C++ 落点 | 职责 |
|---|---|---|
| `pipeline` | `SlamSystemConfig::pipeline` | 应用选择流水线 |
| `trajectory_path` | `SlamSystemConfig::trajectory_path` | 轨迹输出位置 |
| `sensors.lidar.*` | `SlamSystemConfig::voxel_slam.lidar` → `TrajectoryBuilderOptions::lidar` | LiDAR 型号、盲区、抽样步长及扫描相对时间 |
| `frontend.initialization.minimum_imu_sample_count` | `SlamSystemConfig::initialization.minimum_imu_sample_count` → `InitializationOptions::minimum_imu_sample_count` | 初始化 IMU 样本数 |
| `frontend.initialization.gravity_magnitude` | `SlamSystemConfig::initialization.gravity_magnitude` → `InitializationOptions::gravity_magnitude` | 重力幅值，m/s² |
| `frontend.initialization.maximum_acceleration_variance` | `SlamSystemConfig::initialization.maximum_acceleration_variance` → `InitializationOptions::maximum_acceleration_variance` | 加速度方差阈值 |
| `frontend.local_mapping.window_size` | `SlamSystemConfig::local_mapping.window_size` → `LocalMapping::Config::window_size` | 滑窗关键帧数 |
| `frontend.odometry.*` | `SlamSystemConfig::voxel_slam.frontend` → `TrajectoryBuilderOptions` | 窗口、线程、降采样和局部 BA 能力 |
| `mapping.voxel.*` | `SlamSystemConfig::voxel_slam.mapping` → `TrajectoryBuilderOptions` | 体素尺度和最小平面特征值 |
| `optimization.*` | `SlamSystemConfig::voxel_slam.optimization` → `TrajectoryBuilderOptions` | 惯性因子系数和回环 ICP 判据 |
| `backend.loop_closure.*` | `SlamSystemConfig::voxel_slam.loop_closure` → `TrajectoryBuilderOptions` | 回环、三角描述子、门限与触发间隔 |
| `backend.global_mapping.*` | `SlamSystemConfig::voxel_slam.global_mapping` → `TrajectoryBuilderOptions` | GBA 开关、局部窗口和合并尺寸 |
| `storage.*` | `SlamSystemConfig::voxel_slam.storage` → `TrajectoryBuilderOptions` | 会话目录、名称和历史地图选择 |

目前配置边界使用完整、可读的字段名；`TrajectoryBuilderOptions` 中保留少量与原算法同名的实现字段，由 `VoxelSlamSystem` 显式映射。参数的具体公式语义仍须以消费点为准，不能从字段名推断。

## 单文件内部结构

配置区块仍按系统链路分组，模块拥有唯一参数命名空间：

可运行 schema 和当前默认值以 [`voxel_slam_default.yaml`](../../../config/voxel_slam_default.yaml) 为准。此文件的字段均由 `LoadSlamSystemConfig` 识别，未知键拒绝加载；这里不另行维护结构示意，避免形成与运行配置漂移的第二份 schema。

## 重名、重复与兼容规则

- 轨迹输出位置只归应用 I/O；会话地图目录和历史会话只归存储/会话加载，不再用 `save_path` 同时表达多种输出。
- `frontend.odometry.point_downsampling_size_meters` 与 `mapping.voxel.voxel_size_meters` 是不同参数，不因当前数值类似而合并。
- 回环总开关控制回环子系统，三角描述子开关单独控制描述子能力；二者映射到算法中两个独立选项。
- 系统参数只定义在唯一 YAML；不通过多文件继承或复制 profile 表达场景差异。CLI 仅覆盖已明确定义的输入/输出参数，不隐式覆盖算法参数。
- 配置边界的完整命名到算法内部历史字段映射集中维护在 `VoxelSlamSystem`，并通过配置往返与装配测试保护。
- 兼容期应在加载边界集中处理旧键，并对新旧键同时出现的冲突明确报错；不要让算法模块长期携带两套字段名。

## 代码落点与完成条件

- `app/slam_system_config.hpp/.cpp`：应用配置类型、解析、默认值和校验。
- `app/voxel_slam_system.cpp`：将应用参数转换为 `frontend::TrajectoryBuilderOptions`，不在算法模块解析 YAML。
- `atlas/atlas/frontend/trajectory_builder_options.hpp`：Voxel-SLAM 算法选项的最终类型定义；每个字段有唯一消费位置。
- IMU/LiDAR 外参、IMU 噪声和点云测量误差尚未进入应用配置；在坐标系与单位契约得到源码验证后再加入，不从 `extrinsic_R` 等字段名猜测方向。

完成配置重构时，需同步更新解析器、强类型配置、校验、装配、唯一默认 YAML、CLI 文档和配置往返测试。否则文档示意只作为设计，不宣称已经可运行。
