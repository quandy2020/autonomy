# Voxel-SLAM 工程代码重构分阶段路线图

> 本文的进度记录是 2026-10-24 至 25 日的历史快照。2026-10-27 的完整 Oxford Spires 评测、当前误差和复现命令见 [评测记录](../validation/oxford-spires-full/README.md)。当前位姿图仅使用 GTSAM；文中早期的 `PoseGraphFallback` 已删除。

> **实施要求：** 按阶段逐项执行；每阶段形成独立、可审查、可回退的代码切片。进入实现时使用 `superpowers:executing-plans` 按步骤执行。每项使用复选框跟踪，不得将未运行的验证标记为通过。

**目标：** 在保持 Voxel-SLAM 算法行为的前提下，逐步把 Atlas 当前以 `TrajectoryBuilder` 为中心的状态与并发协调拆到职责清晰、可独立验证的模块中，并让 C++ 命名空间与实际源码目录一致。

**架构：** 以 Atlas 当前实现和原始 Voxel-SLAM 源码作为行为基准，先建立可重复的基线与契约，再依次收敛配置/依赖、值模型、前端、地图、优化后端、后台任务和持久化。`Odometry` 作为兼容入口逐阶段委派；领域状态由唯一 owner 持有；异步任务传递不可变快照并通过版本检查提交结果。

**技术栈：** C++17、Eigen、Threads、glog、GTSAM（必选）、AutoMsgs（边界可选）、GTest（测试期）。不引入 ROS 或 PCL 到 Atlas 核心；PCL 原实现所需的窄功能由 Atlas 自有代码提供。

## 实施进度（2026-10-25）

| 阶段 | 状态 | 当前证据/剩余工作 |
|---|---|---|
| 0 基线与源码追踪 | 部分完成 | Oxford Spires 数据仅用于重构过程中的 200 帧前缀诊断；依用户最新要求，当前数值不作为验收基线。正式基线须在全部代码重构完成后，以序列官方标定和匹配的 Hesai 参数重跑；吞吐/内存基线仍缺失。 |
| 1 行为特征测试 | 进行中 | 新增点时间单位、配置非法值、系统生命周期及初始化拒绝非有限/乱序 IMU 的回归测试；还需补 IMU、BA、回环、位姿图和序列化边界测试。 |
| 2 因子图优化依赖与构建边界 | 已完成 | GTSAM 是必选依赖；项目内部文件、类和函数不以第三方库名命名。当前唯一实现为持有 ISAM2 状态的 `PoseGraphSolver`，位姿图输入由 Atlas 类型表达。
| 3 单 YAML 与强类型配置 | 部分完成 | 唯一 YAML、分模块强类型 options、解析/校验/序列化及已暴露参数的 `TrajectoryBuilderOptions` 显式装配已贯通；IMU 测量噪声、两类 bias random walk 和 LiDAR→IMU 旋转/平移已接入配置；92 项测试通过。三角描述子全量选项尚未配置化；真实数据集外参回放仍在验证。 |
| 4 领域值类型与版本契约 | 未开始 | 尚未系统建立 TrajectoryId、MapVersion 等跨模块身份/版本契约。 |
| 5 会话生命周期与流水线协调 | 部分完成 | 生命周期状态机位于 `frontend/trajectory_lifecycle.hpp`，编排器已迁为 `TrajectoryBuilder` 并移除无替换需求的接口层；会话读写已拆到 reader/writer。历史/当前关键帧、滑窗预积分、体素树根/子节点及 FrameWindow 缓存均由 unique ownership 管理；GBA 游标、边集和 submap 缓冲已归 `GlobalMappingState` 所有；阶段编排和结构化收尾结果仍未拆分完成。 |
| 6 前端解耦 | 部分完成 | 初始化实现已迁到目标 `frontend/initialization/`，包含 `StateInitializer`、`InitializationOptions`、`GravityAlignment` 和 `InitializationCriteria`；惯性状态、滤波和预积分已迁到 `frontend/inertial/`；里程计已迁至 `frontend/odometry/`，局部建图源码已迁至 `frontend/local_mapping/`；主编排器已迁到 `frontend/trajectory_builder.*` 并命名为 `TrajectoryBuilder`；共享状态提升到 `frontend/frontend_state.hpp`。`MotionInitializer::Run` 已拆出输入校验、观测准备、BA/预积分器重建、可观测性计算和结果分类，主流程 73 行；初始化地图建立和失败清理由 `map/SlidingVoxelMapOperations` 执行，重切片维持串行行为。BA 构造/迭代仍由 initializer 编排，builder 内部阶段状态、会话持久化、错误/收尾结果及领域所有权也仍未拆完。 |
| 7 地图与会话存储 | 部分完成 | 体素地图、HBA 和旧格式读写存在；`SessionWriter` 与流式 `SessionReader` 已迁入 `map/serialization/`，负责当前姿态文本与关键帧点云格式。目标 `AdaptiveVoxelMap`、版本化快照、新会话文件格式和原子发布未实现。 |
| 8 因子图与增量位姿图优化 | 部分完成 | Eigen/G2O adapter、多求解器接口、能力协商和 factory 已删除；在线与 TopDown PGO 直接调用唯一 `PoseGraphSolver`。ISAM2 状态由求解器实例持有并跨调用更新。六维对角信息已贯通；固定图成本和位姿适配器对照已通过。Oxford Spires 回放暂不作为验收；须待重构完成后评估 backend 的端到端优化效果。
| 9 回环、位姿图与全局任务 | 部分完成 | BTC、PGO/GTSAM、全局优化代码已有；GBA worker 生命周期已迁入 `GlobalMapping` 并复用单 worker `common::ThreadPool`，finish 会持续排空全部局部窗口后执行最终全局优化，后台异常由 `Finalize` 报告。`GlobalMappingRequestBuilder` 负责局部窗口、submap、residual 三类不可变点云/位姿快照构造；`GlobalMappingCommitter` 统一校验当前 pose/map/session 版本和活动关键帧身份，并在完整验证后提交整批位姿；`GlobalMappingState` 拥有 HBA 游标、两阶段边集和 submap 缓冲，并负责版本化提交。全局阶段函数已按职责拆开；TopDown 与在线增量 PGO 均使用 `PoseGraphSnapshot` 在 map 锁外优化，并在版本/身份校验后提交；TopDown 的位姿提交、pose/map 版本递增及体素地图重建处于同一地图锁临界区，在线 PGO 的滑窗与体素地图校正也在 map 锁内完成。submap 不足两个时清空旧第二阶段 PGO 边的 map version 缺口已修复。request/result 的定制队列不是当前串行阶段链所必需；阶段循环仍在 `TrajectoryBuilder`，跨模块 map/session 版本推进审计仍未完成。 |
| 10 多会话持久化 | 部分完成 | 旧会话读写能力存在；`SessionReader` 流式读取姿态和点云，`PreviousSessionLoader` 在临时容器构造成功后才替换现有会话/图边，读取失败不会先清空活动输入，并已覆盖此失败语义。结构化失败结果、版本化格式、校验和、写入原子发布及更完整的损坏输入恢复仍待完成。 |
| 11 目录与命名收敛 | 进行中 | 初始化、惯性、里程计、局部建图、主编排器和共享前端状态已迁入设计目录，旧 `frontend/lidar/` 目录已清理；BTC 代码已移出旧 Lidar/descriptor 层并采用责任目录；单实现的编排接口层已移除；会话写入器和流式读取器已迁至序列化模块。历史/当前关键帧、GBA 子地图、滑窗预积分、体素根/子树和 FrameWindow 缓存均改为 RAII 所有权，算法边界使用局部非拥有视图或引用。`SlidingVoxelTree` 配置现按地图实例创建并由节点树共享只读持有，Li-BA 惯性权重由 optimizer 实例持有，相关进程级变量已移除。活动图和 loop factor 已从 `optimization/voxel/` 迁到 `backend/pose_graph/`，不再保留旧优化目录；`PreviousSessionLoader` 仍负责会话关键帧装配；GBA 算法状态与执行回调仍归 builder；通用图与活动图的共同领域契约尚未收敛。 |
| 12 性能与完整验收 | 未开始 | 缺少固定数据集/真值及可复现精度、吞吐和内存基线；算法等价与性能验收未完成。 |

**本轮进度补充（2026-10-25）：** TopDown PGO 的备用图构建、回环边装配与优化已拆至 `PoseGraphFallback`；跨 session 快照校验和批量位姿提交已拆至 `PoseGraphCommitter`。请求 ID 发放、局部/submap/residual HBA 计算、三类请求快照构造和 HBA 批量位姿提交均已有 backend 边界：分别为 `GlobalMappingOptimizer`、`GlobalMappingRequestBuilder`、`GlobalMappingCommitter` 和 `GlobalMappingState`。`GlobalMappingState` 拥有 HBA 游标、两阶段边集及 submap 缓冲。`RunGlobalMappingWork()` 已拆为六个不超过 60 行的配置/阶段方法；TopDown 与在线增量 PGO 均通过 `PoseGraphSnapshot` 锁外求解、版本校验和锁内提交；`TrySessionLoops()` 已按候选验证、位姿检索和会话判据拆分。在线图优化不再占用 map 锁，结果提交和滑窗/地图校正仍受版本校验和 map 锁保护。`TrajectoryBuilder` 仍拥有阶段循环，跨模块 map/session 版本审计与阶段编排迁移尚未完成。当前全量构建通过；112/112 全量测试结果来自本轮改动前的最近一次运行，本轮未重跑测试。

最近验证：在线增量 PGO 快照拆分后，`cmake --build atlas/build/refactor -j 4` 成功。最新全量测试结果为本轮改动前的 112/112；当前改动后未运行测试。相关 C++ 文件 clang-format 检查通过。`TrajectoryBuilder` 仍有两个超过 80 行的函数：`LioStateEstimation` 和 `Process`。此前状态测试覆盖成功提交推进游标/版本及拒绝过期请求；快照构造测试覆盖局部窗口索引和点云共享、submap 原索引、无点云 residual 过滤及游标越界。此前 Oxford Spires 前缀轨迹及参数消融结果仅保留为临时诊断，不作精度验收。按用户最新要求，全部代码重构完成后才按序列官方标定和匹配 Hesai 参数进行正式数据集评测；完整序列、backend ATE/RMSE、吞吐及内存指标均未验证。

**设计依据：** [当前源码架构与数据流程](../architecture/current-architecture-and-dataflow.md)、[重构评估与缺口](../architecture/refactor-assessment.md)、[重构原则](../architecture/refactor-principles.md)、[目标目录与迁移映射](../architecture/design/project-tree.md)、[文件与类设计](../architecture/design/file-class-design.md)、[唯一 YAML 配置契约](../architecture/design/configuration.md)、[优化公共抽象](../architecture/design/optimization.md)。算法行为参照 `/Users/quandy/Workspace/github/ros2/ros-perception/Voxel-SLAM/VoxelSLAM/src/voxelslam.cpp` 与对应论文章节。

## 全局硬约束

- 最低语言标准为 C++17；新 API、文件和类名遵循 Google C++ 风格及已确认的完整词命名约定。
- 全系统只有 `atlas/config/voxel_slam_default.yaml` 一个 YAML；模块参数在同一文件内分区，不增加 profile、include、继承或复制配置。
- `GTSAM` 是后端优化必选依赖；不得在缺失时静默切换到算法语义不同的回退实现。
- `atlas_voxel` 核心不依赖 ROS、AutoMsgs 或 PCL；AutoMsgs 只位于适配 target；不复制 PCL 的大而全 API。
- `TrajectoryBuilder` 和现有 `VoxelSlamSystem` 调用边界保持兼容。
- 算法阈值、数据关联、坐标约定和序列化格式不得在纯架构重构中顺手调整。
- 每个可变状态有唯一 owner；跨线程输入不可变；位姿更新携带 trajectory/map version 并在明确边界提交。
- 不恢复、覆盖或清理任务开始前已有的无关工作区修改；实现阶段记录初始 HEAD 和 status，只修改计划列明的 Atlas 文件。

## Review Focus

- IMU/LiDAR 时间戳乱序、重复或过晚：输入策略不得污染前端状态（阶段 1、6）。
- 初始化样本不足、持续退化或包含非有限数：不得发布有效轨迹/地图（阶段 1、5、6）。
- 回环/GBA 结果在前端更新期间到达或 source map version 已过期：不得覆盖更新地图（阶段 4、7、9）。
- `Finalize`/`Stop` 时仍有排队或运行任务：按约定 drain/cancel/join，不丢失已承诺持久化的数据（阶段 9、10）。
- 历史 session 损坏、版本不兼容或部分加载失败：活动状态必须原子保持不变（阶段 10）。

## 进入代码前的验证基线

计划执行前应使用一个专用构建目录保存基线，避免覆盖用户已有 build：

```bash
cmake -S atlas -B atlas/build/refactor \
  -DATLAS_BUILD_APPLICATIONS=ON \
  -DATLAS_BUILD_MESSAGE_ADAPTERS=ON \
  -DBUILD_TESTING=ON
cmake --build atlas/build/refactor -j
ctest --test-dir atlas/build/refactor --output-on-failure
```

当前 GTSAM 查找为可选，因此在“必选依赖阶段”完成前，这组命令只能记录当前环境基线，不代表依赖约束已达标。若当前配置或测试失败，先保存完整日志并判断是否为已有问题；不得把已有失败归因于后续阶段。

对可用的固定数据集运行现有回放入口并记录：数据集标识/校验和、唯一配置文件、构建版本、扫描接收数、轨迹有效帧数、终点误差或可用参考轨迹误差、执行时间及峰值内存。没有固定数据集或参考轨迹时，明确标为“未建立基线”，不可用目测替代数值结论。

## 阶段总览与依赖

```text
0 基线与源码追踪
  → 1 行为特征测试
  → 2 构建依赖边界（GTSAM 必选）
  → 3 单 YAML 与强类型配置
  → 4 领域值类型和版本模型
  → 5 生命周期/协调器边界
  → 6 前端解耦
  → 7 地图状态和快照
  → 8 优化抽象与后端图/回环
  → 9 全局任务并发与结果提交
  → 10 会话持久化
  → 11 应用集成、性能和最终收敛
```

后续阶段可以在同一阶段内分多个小 PR/提交；不得跳过前置阶段的验收门，也不得把多个高风险算法模块同时迁移。

## 阶段 0：建立事实基线和代码追踪表

**目的：** 在改代码前区分论文设计、原始实现和 Atlas 当前行为，防止重构遗漏能力或把既有问题误判为回归。

**主要范围：** `atlas/docs/architecture/current-architecture-and-dataflow.md`、`refactor-assessment.md`、`atlas/atlas/frontend/trajectory_builder.cpp/.hpp`、`atlas/CMakeLists.txt`、`atlas/app/`。

**结果产出：**

- 每个阶段的原始函数/线程 → Atlas 调用点 → 状态 owner → 输入输出 → 当前状态（实现/部分/缺失/待验证）映射表。
- 初始 HEAD、工作区 status、构建命令及成功/失败日志、测试清单、可用数据集回放指标。
- 明确尚无基线的性能/精度项和取得基线所需的输入数据。

**执行步骤：**

1. 记录 `git rev-parse HEAD` 与 `git status --short`，保存但不清理已有改动。
2. 从 `replay_main.cpp` 追到 `FileReplaySource`、`VoxelSlamSystem`、`TrajectoryBuilder::Process`，画出现实调用路径。
3. 针对初始化、IMU EKF/预积分、局部 BA、地图更新、BTC、ISAM2/PGO、HBA、会话读写逐项找源码和调用方。
4. 用专用目录执行上面的 CMake/build/ctest 命令，原样记录首个失败目标和日志路径。
5. 对已有数据集执行回放；如果缺数据或没有 ground truth，将缺口写入评估表，不编造指标。

**阶段验收：** 架构图与代码入口可逐项互相追溯；当前失败与未验证项已单独列明；工作区初始状态有记录。此阶段不修改算法实现。

## 阶段 1：建立行为特征测试和可比较输出

**目的：** 先固定可观察行为，再移动所有权。特征测试描述“当前正确行为”，不把尚未确认的论文细节锁死为新规范。

**主要范围：** 现有 `atlas/atlas/*_test.cpp`、`atlas/app/*_test.cpp` 和 CMake 测试注册。

**结果产出：** 对每条待拆链路有至少一个窄测试，失败时能定位具体模块；形成输入/输出样例或固定种子 synthetic data。

**执行步骤：**

1. 扩展现有 `initialization_test`，固定初始化成功、样本不足、退化/无效输入的状态与输出契约。
2. 扩展 `inertial_state_filter_test`、`inertial_preintegrator_test` 和 `point_covariance_estimator_test`，固定时间顺序、有限值、偏置/协方差重置及数值结果容差。
3. 扩展 `inertial_bundle_adjuster_test`，覆盖窗口不足、成功优化、失败输入以及优化前后状态的有限性。
4. 扩展 `triangle_loop_detector_test`、`triangle_search_loop_test` 和 `pgo_edges_test`，覆盖候选拒绝/接受、约束方向、重复边和无效协方差。
5. 扩展 `hierarchical_bundle_adjustment_test` 与 `voxel_map_serializer_test`，覆盖输入无效、正常输出、空数据和读写往返。
6. 为端到端回放记录固定统计项，不把机器绝对耗时作为跨机器硬阈值；先建立同一机器重复运行的方差区间。

**验证命令：**

```bash
cmake --build atlas/build/refactor -j
ctest --test-dir atlas/build/refactor --output-on-failure
```

**阶段验收：** 新测试在当前代码下通过；数值容差有坐标系/单位依据；测试没有依赖不稳定线程时序或随机种子。若某条预期行为尚未证实，测试名和注释须标为待确认，不以失败测试强行改变算法。

## 阶段 2：收敛构建依赖和 CMake 边界

**目的：** 固定后续模块可依赖的真实构建契约，确保 GTSAM 必须存在且核心不被 ROS/PCL/AutoMsgs 拖入。

**主要范围：** `atlas/CMakeLists.txt`、`atlas/cmake/functions.cmake`、与 Cartographer 一致的 `atlas/cmake/modules/` 文件树，以及相关 adapter 源文件。

**结果产出：** GTSAM 配置时必需、target 名兼容集中管理、AutoMsgs 仍可关闭、核心依赖图无 ROS/PCL；库/adapter/app/test 目标边界明确。

**执行步骤：**

1. 将 GTSAM 查找调整为 required，集中兼容已支持的导出目标名；移除“无 GTSAM 时编译不同算法”的静默路径。
2. 保证 GTSAM 头文件和类型留在 adapter/私有实现，公共优化头文件不暴露 `gtsam::*`。
3. 检查 `atlas_voxel` 的公开 link/interface；AutoMsgs 只由 message adapter/application 按需链接。
4. 检查 CMake source/link 与源码 include，确保没有新增 PCL 或 ROS 查找和 include。
5. 更新 README 的“GTSAM 必选”说明和核心/应用构建命令。

**验证：**

- GTSAM 安装环境：配置、`atlas_voxel`、应用及测试全部成功构建。
- 无 GTSAM 环境：CMake 配置在清楚的依赖诊断处失败，不进入隐式 Eigen/伪实现。
- AutoMsgs 关闭且 applications/tests 关闭：核心目标仍能构建。
- `rg -n 'find_package\(PCL|pcl::|#include <pcl/|find_package\(rclcpp|#include <rclcpp/' atlas` 无核心匹配；手动审查适配目录例外。

**阶段验收：** CMake 依赖图符合约束；README 和 CMake 结果一致；构建失败路径明确。不得以此阶段重写 GTSAM 因子或改变解算策略。

## 阶段 3：单一 YAML 与强类型配置收敛

**目的：** 使唯一配置文件、应用配置类型、算法 options、验证逻辑和真实消费点形成闭环。

**主要范围：** `atlas/config/voxel_slam_default.yaml`、`atlas/app/slam_system_config.hpp/.cpp`、`voxel_slam_system.cpp`、`trajectory_builder_options.hpp`、配置文档。

**结果产出：** 一个可加载的 YAML 文件；字段—C++ 成员—消费点映射表；校验规则和清楚的错误位置；配置变更不再产生复制 profile。

**执行步骤：**

1. 以当前 default YAML 和 `SlamSystemConfig`/`TrajectoryBuilderOptions` 建字段清单，给每项补单位、默认值、约束、唯一 owner、调用消费位置。
2. 只对源码可证实的参数重命名；无法证明 `imu_coef`、`degrade_bound`、`icp_eigval`、`jud_default`、`ratio_drift` 语义的项先保留旧名字并记录待确认，不按字面猜测。
3. 将分散的 `voxel_*` app 字段整理成按模块分组的强类型 options；由 app 边界进行一次显式转换，算法模块不读 YAML。
4. 更新现有 `config/voxel_slam_default.yaml`，将传感器、frontend、mapping、optimization、backend、storage 参数放入单文件有序分区；不增加 include/profile 文件。
5. 为默认值、合法值、缺失可选项、未知键、重复键、非法数字和往返保存增加/更新 `slam_system_config_test`。
6. 明确当前解析器支持的 YAML 子集；只要它能无歧义加载单文件 schema，就不额外加入 YAML 依赖。若需扩展语法，先单独论证依赖和兼容性。
7. 检查 `atlas/config/`：只保留一个 `.yaml`；数据集说明继续使用 Markdown，不生成第二份 YAML。

**验证命令：**

```bash
cmake --build atlas/build/refactor --target slam_system_config_test replay -j
./atlas/build/refactor/app/slam_system_config_test
./atlas/build/refactor/app/replay --config atlas/config/voxel_slam_default.yaml --dataset <dataset_directory> --output <output_directory>
```

**阶段验收：** `rg --files atlas/config -g '*.yaml'` 只返回 `voxel_slam_default.yaml`；配置字段全部能追到 options 与消费者；空 YAML、重复配置副本和未实现字段不再误导使用者。无可用 dataset 时只完成 parser 单测，不声明端到端验证通过。

## 阶段 4：建立领域值类型、身份和版本契约

**目的：** 在拆业务算法前，统一 trajectory/node/submap/constraint 的身份、坐标方向和异步版本数据。

**主要范围：** `atlas/atlas/backend/pose_graph/`、`atlas/atlas/map/`、`atlas/atlas/frontend/` 的值类型和消息头文件。

**结果产出：** 小而稳定的强类型 ID、姿态/约束方向约定、不可变观测值、trajectory/map version；不引入没有消费者的模型。

**执行步骤：**

1. 对照原始 `ScanPose`、`Keyframe`、session id 和现有 PGO 边字段，确定哪些是同一实体、哪些不能合并。
2. 定义最小必需的 `TrajectoryId`、`NodeId`、`SubmapId`、`MapVersion`，包含比较、有效性及哈希；避免裸整数 ID 跨域混用。
3. 定义 `PoseConstraint` 的 source/target 顺序、变换方向、单位和不确定度表示；所有输入边验证有限值、正定/可用协方差。
4. 定义只读节点观测和 map snapshot 的所有权语义；位姿优化更新独立于不可变点云观测。
5. 对 ID 不相混淆、约束方向、无效输入拒绝、版本比较写独立测试。

**验证：** 新类型单测；现有 `pgo_edges_test` 和回环测试仍通过；序列化前后身份和变换方向一致。对同一组固定姿态，用正反方向 composition 测试确认约定。

**阶段验收：** 领域 ID 不存在隐式交叉转换；任何跨线程更新可以指出 source version；值模型只依赖 Atlas/C++17，不泄漏 GTSAM、ROS、AutoMsgs 或路径字符串。

## 阶段 5：抽离会话生命周期与流水线协调

**目的：** `Odometry` 保持公共兼容性，但不再同时拥有系统生命周期、所有模块状态和后台线程。

**主要范围：** `atlas/atlas/frontend/trajectory_builder.hpp/.cpp`，仅在职责确定后新增生命周期/协调器/消息文件。

**结果产出：** 明确状态机和合法迁移；`Odometry` façade 只校验/委派/维持既有接口；协调器有明确创建、输入、结束和输出顺序。

**执行步骤：**

1. 根据真实行为画出 `Uninitialized → Initializing → Tracking/Degraded → Finalizing → Stopped` 状态转移和失败返回，不把论文中的重定位能力混入状态机。
2. 为重复 Start/Stop、初始化失败、异常输入和终止中追加扫描定义既有兼容行为。
3. 建立一个最小 coordinator 只负责模块连接、调用顺序和输出 drain；不搬 EKF、BA、BTC/HBA 算法实现。
4. 让 `VoxelSlamSystem` 直接调用 `TrajectoryBuilder`，并保持应用生命周期和结果类型稳定。
5. 给状态机每条允许/拒绝转换写单测；给 façade 写输入顺序和输出兼容测试。

**验证：** 新增 `voxel_slam_lifecycle_test` / `voxel_slam_pipeline_test`；运行前端现有初始化、IMU、里程计和局部 BA 测试；单次数据集回放的扫描接收数和轨迹输出与基线一致。

**阶段验收：** coordinator 没有复制模块内部算法状态；每个状态字段有唯一 owner；老入口未被删除；错误路径可确定停止系统且不遗失 finalization 错误。

## 阶段 6：逐项解耦前端

**目的：** 保持实时路径行为和时延，分离初始化、惯性状态、scan-to-map 里程计、局部滑窗 BA。

**主要范围：** `frontend/initialization/`、`frontend/inertial/`、`frontend/odometry/`、尚待迁移的 `frontend/local_mapping/` 及其 Voxel-SLAM 编排接线。

**结果产出：** 模块使用强类型输入/输出和 options；滑窗/初始化缓存不再由总编排器直接读写；新增状态 owner 和依赖图。

**执行步骤：**

1. 一次只迁一个阶段，先选择已有测试覆盖最好、和异步线程共享最少的模块。
2. 将初始化窗口与成功/失败判定封装在 initialization owner；保留原始重力/收敛公式。
3. 将惯性传播/预积分状态封装在 inertial owner；由明确同步数据输入，保留时间戳和 bias reset 行为。
4. 将局部 scan-to-map 上下文封装到 odometry owner；地图只读借用/快照，不从 backend 反向读私有状态。
5. 将固定容量滑窗、BA 变量和边缘化输出归 local mapping owner；输出不可变 keyframe/node 消息。
6. 每个小切片完成后先删重复旧状态字段，再移动下一模块，避免双写状态长期共存。

**验证：** 每次只跑对应 `initialization_test`、`inertial_state_filter_test`/`inertial_preintegrator_test`、`point_covariance_estimator_test`、`inertial_bundle_adjuster_test`；每个提取后跑 `voxel_slam_pipeline_test`；数据集回放比较轨迹差异、有效帧数和实时耗时。改变输出超过预先记录容差即停在该切片诊断。

**阶段验收：** 依赖无反向引用；每类状态只写于一个模块；前端热路径无全局地图深拷贝、无等待 GTSAM 全局优化、无逐扫描创建线程。

## 阶段 7：收敛地图 owner、快照和点云能力

**目的：** 分开地图几何数据、索引生命周期、序列化和展示投影，替换目前跨路径可变地图状态访问。

**主要范围：** `atlas/atlas/map/`、`map/serialization/`、`sensor/` 以及 `voxel_slam.cpp` 地图调用。

**结果产出：** 地图读/写边界、不可变 map snapshot、版本化替换/更新接口；点云、体素采样和近邻查询继续不依赖 PCL。

**执行步骤：**

1. 静态追踪 `feat_map_`、滑动树、关键帧/点簇缓冲的读写方和锁；记录锁顺序与每个引用的生命周期。
2. 指定唯一地图 owner；为 odometry 提供只读查询，为写入提供窄的更新/commit API。
3. 把回环/GBA 所需地图输入封成只读 snapshot；创建快照的复制/共享成本必须有测量数据。
4. 将 PCL 移植中实际使用的点云容器、filter、近邻查询操作限于 Atlas 接口，不实现 PCL 兼容大 API。
5. 序列化只接收稳定地图/会话值，不在地图类中读写文件；失败读取先构造临时完整对象，再原子提交。
6. 为地图更新冲突、空地图、边界体素 key、只读快照稳定性、序列化往返写测试。

**验证：** `hierarchical_bundle_adjustment_test`、`voxel_map_serializer_test`、体素查询新测试与 `voxel_slam_pipeline_test`；用大/小两组固定点云比较查询和 snapshot 创建耗时、内存拷贝字节数。

**阶段验收：** backend 没有对活动前端 map 的裸可变引用；点云算法和容器没有 PCL 头/API；快照版本与写入版本可追溯。

## 阶段 8：优化公共接口与 Backend 图/回环

**目的：** 将问题建模与求解器实现隔离，同时保持 GTSAM 的必选能力和 Voxel-SLAM 的增量/批量求解语义。

**主要范围：** `backend/pose_graph/`、`backend/loop_closure/`、GTSAM CMake adapter。

**结果产出：** Atlas 因子图输入和求解结果；唯一增量位姿图求解类；PoseGraph 唯一图状态 owner；BTC 产生已验证约束，不直接改地图。

**执行步骤：**

1. 保留在线 ISAM2 与 TopDown PGO 的实际调用语义；已新增 Atlas 因子图与独立原生 ISAM2 固定图对照，逐 pose 比较旋转/平移及最终测量成本。该测试验证适配器一致性，不替代旧版完整 Voxel-SLAM 基准。
2. **已完成基础收敛：** `PoseGraphSolver::Update` 返回状态、update 次数和诊断；`CalculateEstimate` 按需返回位姿与最终成本。多求解器 factory 与能力协商已删除。
3. **已完成唯一增量求解实现接线：** `PoseGraphSolver` 直接持有 ISAM2 状态；核心 CMake 要求 GTSAM。在线 PGO 使用 Atlas `PoseGraphSolver`；TopDown 的局部备用图仍由 backend `LoopPoseGraph` 执行，并已从 `TrajectoryBuilder` 拆至 `PoseGraphFallback`。Eigen/G2O 优化器不属于当前实现范围。
4. **已完成实现：** 活动图状态归 backend；求解器只接受新增节点和因子批次并保护失败恢复；fallback 的图组装及快照位姿写回也归 backend。跨 session/map 来源版本验证仍需整体审计；真实数据集端到端精度评估按用户要求延后至全部重构完成之后。
5. 抽出 BTC 候选产生和几何验证的窄接口；把已验证测量作为 `PoseConstraint` 输出。
6. 保持原回环门限、关联排序和校正算法不变；将涉及阈值的变化留到单独算法变更。

**验证：** `pgo_edges_test`、`triangle_loop_detector_test`、`triangle_search_loop_test`、增量位姿图求解结果和 PoseGraph version tests；固定图适配器对照通过。真实数据集 ATE/RMSE 与性能待非 ROS 回放完成后记录。

**阶段验收：** 公共头无 GTSAM 类型；唯一增量求解类可构建；不支持的因子显式报错；回环验证不直接变更 frontend/map。

## 阶段 9：全局 Mapping 异步任务和线程池生命周期

**目的：** 把当前 `Odometry` 持有的 GBA worker、快照缓存、停止标志和结果合并转移到唯一后台任务 owner。

**主要范围：** `backend/global_mapping/`、`common/thread_pool.*`、HBA 实现接线以及协调器的提交点。

**结果产出：** 有界请求/结果流程、明确排队/拒绝/取消/停止语义、只读快照、版本化校正提交；不再由算法类自行管理孤立线程。

**执行步骤：**

1. 用现有 `ThreadPool` 行为核对所需队列容量/关闭语义；仅补它缺少的通用能力，不在通用线程池加入 SLAM 逻辑。
2. 定义 `GlobalMappingRequest/Result` 最小值类型：任务 ID、trajectory/map source version、const 快照、状态、结果和错误。
3. 将单会话 GBA/HBA 计算封装为同步算法调用；后台 service 负责排队、生命周期和提交协调。
4. 规定队列满时的行为、Finish/Stop 时 drain 或 cancel 的顺序、异常/失败上报以及线程 join 时点。
5. 提交时校验 source version；过期结果记录并丢弃，不覆盖新状态。
6. 增加调度、并发结束、队列满、取消、失败、旧版本结果和重复停止测试。

**验证：** 新增 `global_mapping_lifecycle_test`；重复运行并发测试以发现竞态；运行 `hierarchical_bundle_adjustment_test` 和 `voxel_slam_pipeline_test`；必要时在独立构建中使用 ThreadSanitizer，不与性能基准混跑。

**阶段验收：** finalize 不泄漏或遗失已承诺保存的数据；所有任务可 join；回调提交只在短临界区操作；前端无等待 GBA 的同步点。

## 阶段 10：会话持久化与多会话恢复

**目的：** 让会话文件版本、加载失败、原子恢复和多会话连通状态由持久化/全局建图边界拥有。

**主要范围：** `atlas/atlas/frontend/trajectory_builder_session.cpp`、`backend/global_mapping/previous_session_loader.*`、`map/serialization/`。

**结果产出：** 明确格式版本和兼容策略；失败加载不污染 active state；保存只发布完整文件；旧格式导入路径与新格式写入路径分离。

**执行步骤：**

1. 固化当前 `alidarState.txt`/`.bin` 旧数据字段与坐标约定，建立只读样本或 fixture。
2. 设计最小 session header/版本信息，明确哪些数据可重建、哪些必须持久化；避免序列化 GTSAM 对象或内存布局。
3. 保存到临时文件并 flush/close 后原子 rename；不完整文件不覆盖上次有效会话。
4. 读取到临时结果并完成版本、长度、finite value、索引引用校验后再一次性发布到活动系统。
5. 多会话导入使用完整 SessionId；检测重复 session、缺失引用、失配坐标系和部分文件失败。
6. 为截断/损坏/旧版本/空会话/重复导入/保存中断写 fixtures 和回归测试。

**验证：** `voxel_map_serializer_test`、新增 `session_persistence_test` 和 `previous_session_loader_test`；保存—加载 round trip 后比较 session/node IDs、位姿、地图统计和约束数量；损坏输入确认 active map/version 保持不变。

**阶段验收：** 加载失败是原子失败；成功保存不产生半文件替换；旧格式只读导入，不再由新代码写出旧格式，除非兼容要求明确批准。

## 阶段 11：应用、AutoMsgs 与命令行集成

**目的：** 证明重构后的模块可以由真实 app 组装和回放，传输格式不进入核心算法。

**主要范围：** `atlas/app/`、`atlas/atlas/sensor/`、CMake adapter/application targets、`atlas/README.md`。

**结果产出：** 单一 `replay` 入口；单配置参数装配；AutoMsgs 转换仍在边界；可复现的数据回放命令和输出说明。

**执行步骤：**

1. `VoxelSlamSystem` 只负责 app 生命周期和将 options 传给算法 facade，不参与地图/图的私有状态维护。
2. 保留现有 CLI：配置文件、dataset、trajectory、output 参数；只有完成代码和文档接线后才新增 CLI 选项。
3. AutoMsgs adapter 转换为 Atlas-owned sensor values；核心 public headers 不包含 protobuf 类型。
4. 确保单 YAML 路径默认唯一；同一输入的配置快照可在日志/结果元数据中识别。
5. 更新 README 的构建、回放、输出工件说明，并链接唯一配置和 docs 导航。

**验证：** `slam_system_config_test`、`voxel_slam_system_test` CTest；验证 AutoMsgs on/off 两种支持的构建方式；固定数据集从 CLI 输入到轨迹/地图输出端到端成功。

**阶段验收：** 应用行为不要求 ROS；AutoMsgs 消息不穿过算法边界；CLI/README/config 说法与实际 executable 参数一致。

## 阶段 12：性能、全量验证和兼容层收尾

**目的：** 对比基线确认模块化没有让实时性能、数值结果或持久化可靠性退化，并清理迁移期间临时状态。

**主要范围：** 全 Atlas 改动、CMake 源列表、公共头/API、文档和测试。

**结果产出：** 可审查的阶段变更集、构建/测试矩阵、数据集回放报告、性能对比、剩余已知缺口；没有未引用的 compatibility wrapper/重复配置。

**执行步骤：**

1. 清点旧字段、旧接口、重复缓存和 façade 转发；只有所有调用点已迁移且兼容窗口结束，才单独移除旧接口。
2. 运行完整构建与 CTest，并运行支持的 core-only、application、AutoMsgs build matrix。
3. 使用阶段 0 同一数据集/机器/参数复跑轨迹、地图和性能指标；报告均值/方差或 p50/p95，不只报单次总时间。
4. 用 profiler/计时点检查前端每帧耗时、地图快照成本、回环/GBA 队列延迟、峰值内存；没有证据不声称“高性能已达标”。
5. 静态检查 C++17 设置、GTSAM required、单 YAML 数量、PCL/ROS 核心依赖、过期文件引用和所有 docs 内链。
6. 更新当前架构评估表，将完成/部分/未验证状态与代码路径、测试名称、数据集报告对应起来。
7. 复查最终 diff，只包含本阶段批准范围；保留任务开始前已有的其他模块变更。

**最终验证命令：**

```bash
cmake -S atlas -B atlas/build/refactor \
  -DATLAS_BUILD_APPLICATIONS=ON \
  -DATLAS_BUILD_MESSAGE_ADAPTERS=ON \
  -DBUILD_TESTING=ON
cmake --build atlas/build/refactor -j
ctest --test-dir atlas/build/refactor --output-on-failure
```

另执行核心-only 配置/构建、缺 GTSAM 时预期配置失败、固定数据集 replay、静态依赖扫描与 Markdown/YAML 文件清单检查。每项报告命令、环境、结果、未覆盖边界；任何未执行项标为“未验证”。

**最终验收：** 所有核心回归测试通过；固定数据集在已定义容差内；性能变化有数据；GTSAM 为必选；只有一个 YAML；核心无 ROS/PCL/AutoMsgs 泄漏；代码、配置、文档和架构评估互相一致。达不到其中任一项时，报告具体阻塞，不以目录整理完成宣称重构完成。

## 每阶段统一交付模板

每个阶段结束时，在 PR/交付记录中填全以下内容：

```text
阶段/目标：
修改文件与责任：
代码产出：
新增或更新的测试：
实际运行的命令：
验证输出/数据：
兼容性和性能对比：
未验证项与风险：
回退点：
下一阶段入口条件：
```

**阶段不可合并条件：** 验证未执行却标记通过；模块重复持有可变状态；算法结果变化没有差异报告；配置文档出现第二个 YAML；GTSAM 缺失时静默改变算法；或工作区无关修改被覆盖。
