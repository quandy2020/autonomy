# Voxel-SLAM Atlas 重构评估与结构缺口

## 审查范围

对照原始仓库 `/Users/quandy/Workspace/github/ros2/ros-perception/Voxel-SLAM/` 和 Atlas 目录中的源码。原版主要算法编排位于 `VoxelSLAM/src/voxelslam.cpp`；Atlas 算法入口为 `atlas/atlas/frontend/trajectory_builder.hpp/.cpp`，应用入口位于 `atlas/app/replay_main.cpp` 与 `atlas/app/voxel_slam_system.*`。本文件描述静态源码结构，不代表本轮已经运行端到端数据集。

## 总体判断

Atlas 的目录分层方向合理：传感器、初始化、里程计、局部建图、体素地图、回环、位姿图、全局建图和应用消息适配均有独立目录；核心算法不要求 ROS，ROS/数据格式转换可放在外围。相较原版把大部分实现集中在一个 ROS 2 源文件中，可读性和可替换性已有明显改善。

但当前仍属于**文件层模块化，部分职责尚未达到类/状态/线程层解耦**。`TrajectoryBuilder` 持有 `SensorSync`、IMU EKF、初始化、局部地图/窗口缓冲、BTC detector、位姿图、关键帧、历史会话、GBA 线程和会话生命周期；方法包括处理扫描、触发局部 BA、提交回环关键帧、应用回环地图更新、GBA worker、top-down PGO 和会话保存等。历史/当前关键帧、GBA 子地图、滑窗 IMU 预积分对象、体素地图根/子节点及 `FrameWindow` 缓存现由 `std::unique_ptr` 管理；算法边界创建短期非拥有视图，线程和大量阶段状态仍集中在 builder。结果是高层编排器仍知道过多内部数据结构，模块测试和并发边界较难独立验证。

`GlobalMapping` 已接管 GBA worker 生命周期、唤醒、finish/drain 和 join，并通过单 worker `common::ThreadPool` 串行执行；finish 会排空所有局部窗口。`GlobalMappingRequestBuilder` 在调用方持地图锁时把局部窗口、submap、residual 输入复制为值快照并共享不可变点云；`GlobalMappingResult` 校验 request id、来源版本、session、位姿状态和 edge 形状；`GlobalMappingCommitter` 再按当前版本和活动关键帧身份完整校验后提交整批位姿。`GlobalMappingState` 现拥有 HBA 游标、两阶段 PGO 边集和 submap 缓冲，并封装三类 HBA 结果提交及 pose/map 版本推进。HBA 的约束输出现为 `common::SessionPoseConstraints`，由 backend 转成 `PgoEdges`。TopDown 与在线增量 PGO 均由 `PoseGraphSnapshot` 提供隔离输入并在 map 锁外求解；通过版本和关键帧身份校验后再提交，在线图优化不再长期占用 map 锁。TopDown 位姿提交、pose/map 版本递增和体素地图重建处于同一地图锁临界区。显式 request/result 消息队列对当前单 worker 串行阶段链没有必要；跨 HBA/PGO 仍是两种领域提交契约，map/session 版本推进路径还需完整审计。`TrajectoryBuilder` 仍负责阶段切换和任务循环，跨模块版本审计未完成。未接入产品的 `BackendRunner`/同步 `PoseGraph` 薄封装已清理。

## 功能链映射

| 能力 | 原始源码证据 | Atlas 代码位置 | 静态评估 |
|---|---|---|---|
| 消息输入与回放 | `VoxelSLAM/src/voxelslam.cpp` 与 PointCloud2 插件 | `app/voxel_slam_message_adapter.*`、`app/file_replay_source.*`、`app/replay_main.cpp` | AutoMsgs 转换和文本数据集读取均位于应用边界；核心算法使用 Atlas sensor 类型 |
| IMU 状态传播/预积分 | `ekf_imu.hpp`、`preintegration.hpp` | `frontend/inertial/inertial_state_filter.*`、`inertial_preintegrator.*`、`inertial_state.hpp` | 已迁到目标目录并采用完整类型名；时间同步和偏置重置语义仍需数据集验证 |
| 动态初始化 | `voxelslam.cpp` 中 `Initialization`、粗到细 BA 与成功判定 | `frontend/initialization/` | 已独立成目录；状态及失败重试仍由主流水线协调 |
| 高频 LIO | `lio_state_estimation()` | `frontend/odometry/` | 算法模块独立；`Odometry` 仍编排状态和匹配地图 |
| 局部滑窗 BA | 原始主循环、BA 因子和窗口缓冲 | `frontend/local_mapping/`，专用路径也在 `voxel_slam.cpp` | 有组件，但需明确通用 `LocalMapping` 与 Voxel-SLAM 专用窗口路径是否双轨、实际哪条启用 |
| 自适应体素/点簇 | `voxel_map.hpp`、`feature_point.hpp` | `map/` | 职责拆分较好；地图实例、锁和快照所有权仍由上层管理 |
| BTC 回环 | `BTC.cpp`、`thd_loop_closure()` | `backend/loop_closure/triangle/` | 描述子模块独立；候选队列、会话检索、阈值和校正仍集中在主编排类 |
| 位姿图优化 | 原始 `build_graph()` / ISAM2 及多会话边 | `backend/pose_graph/`、`backend/global_mapping/` | 活动 PGO 图和增量位姿图求解类均归后端（实现使用 GTSAM）；并列的无调用通用图 facade 已移除，跨 session/map 来源版本和旧/新图数值对照仍待验证 |
| 全局 HBA | 原始 bottom-up 和 top-down HBA 流程 | `map/hierarchical_bundle_adjustment.*`、Voxel-SLAM worker | 算法文件独立，但 GBA 生命周期和结果合并仍集中在 `Odometry` |
| 多会话持久化 | 原始历史会话与全局容器 | `trajectory_builder_session.cpp`、`map/serialization/session_writer.*`、`session_reader.*`、`backend/global_mapping/previous_session_loader.*` | 文件读写与历史会话装配已分层；需继续增加结构化失败结果、版本迁移、原子写入及损坏数据回滚验证 |
| 示例和数据回放 | ROS launch/config + ROS bag | `app/`、`config/voxel_slam_default.yaml` | 当前不提供输入转换工具；示例入口应说明回放数据格式、配置、可执行入口和输出，避免链接已移除的工具 |

表中的“需验证”是源码审查缺口，不等同于已确认的功能缺陷。应通过逐条调用链和针对性测试确认。

## 现有数据流和结构判断

```mermaid
flowchart LR
  A[应用消息/数据回放] --> B[SensorSync]
  B --> C[初始化]
  C --> D[IMU EKF + scan-to-map]
  D --> E[滑窗 LiDAR-IMU BA]
  E --> F[关键帧队列]
  F --> G[BTC 检索与验证]
  G --> H[PGO/跨会话对齐]
  F --> I[关键帧 BA 与子地图]
  H --> J[局部地图校正/重建]
  I --> K[全局 HBA]
  H --> K
  K --> L[轨迹/地图/会话持久化]
```

优点：数据链覆盖初始化、实时估计、局部优化、长期回环、多会话与全局细化；地图、传感器类型和优化算法有可定位的目录；应用层可承接格式转换，核心不依赖 ROS。

风险：`Odometry` 持有太多可变状态；回环位姿校正与局部地图重建跨线程共享状态；GBA 算法仍在 `TrajectoryBuilder` 直接操作可变地图缓冲，尚无版本化请求/结果与提交边界；同一能力存在通用模块与专用实现时，可能产生未使用的抽象或两条不一致调用路径；示例目录和“从数据到结果”的入口说明不够集中。

## 建议的重构顺序

1. **先写清真实调用图**：逐项记录初始化、EKF、局部 BA、BTC、ISAM2、HBA 和保存/恢复的入口、状态 owner 和线程上下文。
2. **定义不可变消息和 session 状态机**：把扫描结果、边缘化关键帧、已验证回环、全局校正、完成请求做成明确数据边界，并带 session/map version。
3. **拆出协调器，不搬算法**：保留 `Odometry` 作为既有接口 facade；将顺序处理、生命周期、模块装配放入 coordinator，算法类继续拥有本领域状态。
4. **把后台全局优化封装完整**：当前已把串行 worker 生命周期、finish/drain、失败上报和线程 join 放入 `GlobalMapping`；`GlobalMappingRequestBuilder` 构造局部/submap/residual 值快照并共享不可变点云；`GlobalMappingCommitter` 负责 HBA 与 residual 结果的版本/身份检查和整批位姿提交；`GlobalMappingState` 拥有 HBA 游标、边集和 submap 缓冲，并执行版本化结果提交。TopDown PGO 由 `PoseGraphCommitter` 校验多 session 快照后整批提交。仍需将阶段循环从 `TrajectoryBuilder` 移出，并完成跨模块 session/map 版本推进审计。
5. **清理重复层**：已删除无产品调用的通用 `BackendRunner`/同步 `PoseGraph`；后续审计聚焦 `TrajectoryBuilder` 与 `GlobalMapping` 的任务/缓冲所有权边界。
6. **整理例子布局**：文档以任务流程组织：单会话离线回放、多会话、前端-only、参数调试；每项链接确切配置和命令。只有新增独立示例程序时再创建 `examples/` 代码目录。
7. **逐片验证**：先覆盖状态机和消息版本，再验证前端输出、loop correction 同步、GBA drain/finalize、会话加载失败原子性，最后跑对应数据集。

## 暂不建议的改动

- 不要按论文五个名词机械地新增五个空 facade；应以状态所有权和变化原因确定类边界。
- 不要让回环线程直接改写正在被前端读取的体素地图；采用版本化结果并在同步边界提交。
- 不要在架构重构同一提交里更换 BA、BTC 或 HBA 算法，避免行为差异与结构差异难以分辨。
- 不要新增 ROS 作为 Atlas 核心依赖；ROS bag 转换保持为外围工具。
- 不要仅因文件数量少就把初始化和局部建图强行合成一个类，也不要仅因论文章节分开就拆出没有独立状态/接口的类。

## 文档示例和目录布局

当前没有独立的 `atlas/examples/` 代码目录，也不提供数据转换、评测或可视化脚本。建议先让 `atlas/docs/README.md` 做入口索引，并按使用任务链接已有的 `app/` 程序和唯一配置 `config/voxel_slam_default.yaml`。示例文档至少列出输入数据格式、执行目标、配置文件、输出轨迹/地图位置和外部数据准备方式。不要把 launch 文件复制到 Atlas 核心；若日后提供独立 demo app，再建立与算法库分离的 `examples/`。

