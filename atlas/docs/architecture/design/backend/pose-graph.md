# Backend 位姿图与约束管理

## 模块边界

活动 Voxel-SLAM 图由 `LoopPoseGraph` 独占节点、约束、回环门控、快照和版本化提交，并将优化问题适配为公共 `FactorGraph` 后调用 `PoseGraphSolver`。先前未接入产品调用的同步 `PoseGraph`/`BackendRunner` 薄封装已移除，避免并列维护两份图状态。LoopClosure 应只提交几何验证后的约束；GlobalMapping 在历史 session 对齐后提交跨会话关系。活动图当前仍由 `TrajectoryBuilder` 持有，尚未迁入独立的 global graph owner。

PoseGraph 不保存完整扫描点云，不负责 BTC 查询、不负责 HBA 八叉树，也不暴露 GTSAM `Values`、`ISAM2` 或第三方因子到公共头文件。目标中的图状态变化必须可审计：约束来源、端点、测量方向、协方差/信息矩阵、验证状态和插入版本均有明确定义。

## 当前目录与文件名

```text
backend/pose_graph/
├── pose_node.hpp
├── prior_factor.hpp
├── between_factor.hpp
├── factor_graph.hpp/.cpp
├── pose_graph_solver.hpp/.cpp
├── loop_factor.hpp/.cpp
└── loop_pose_graph.hpp/.cpp
```

`PoseGraphInterface` 不是默认文件：当前只有一个图状态 owner，不建立无消费者的接口。位姿图领域状态归 backend；`PoseGraphSolver` 只负责将 Atlas 因子图转换为 GTSAM 因子并求解。

## 领域类型

| 类型 | 字段/语义 | 命名规则 |
|---|---|---|
| `PoseConstraint` | `from_node_id`、`to_node_id`、`relative_pose`、协方差/信息、来源、验证元数据 | 变换方向必须在类型注释中固定；不能只叫 `edge` 或 `measurement` |
| `ConstraintSource` | odometry、loop closure、cross-session alignment 等来源 | enum class，拒绝裸整型来源 |
| `ConstraintStatus` | proposed、verified、accepted、rejected 等生命周期 | 仅在多阶段状态真实存在时使用；已验证约束才可送入求解 |
| `PoseGraphNode` | 全局 ID、session/keyframe ID、初值、当前估计、固定状态 | 点云和描述子通过外部不可变快照引用，不嵌入图节点 |
| `PoseGraphVersion` | 每次图结构或位姿提交递增的版本 | 优化请求和结果都携带版本，拒绝旧结果覆盖新图 |
| `PoseGraphResult` | 节点位姿更新、源版本、终止状态和诊断 | 不以就地修改图的隐式副作用返回结果 |

位姿图当前直接使用 `std::int64_t` 表示节点身份，避免为同一整数值引入多套无转换规则的 ID 类型。Session-local ID 只能在 `SessionId + local_index` 配对后解析。

## 主要文件职责

| 文件 | 主类型 | 职责 | 不负责 |
|---|---|---|---|
| `pose_constraint.hpp/.cpp` | `PoseConstraint` | 构造并校验相对位姿约束，统一协方差/信息与方向约定 | 优化求解 |
| `pose_graph.hpp/.cpp` | `PoseGraph` | 添加节点/约束、查询快照、创建优化请求、版本检查和提交结果 | GTSAM 细节、点云生命周期 |
| `pose_graph_options.hpp` | `PoseGraphOptions` | 图优化频率、固定基准、边门限和增量策略 | solver-specific 参数对象 |
| `pose_graph_result.hpp` | `PoseGraphResult` | 给上层返回一致的、带版本优化结果 | 直接改 frontend 状态 |
| `constraint_validation.*` | `ConstraintValidation` | 确认端点存在、变换有限、协方差正定/可修复、无重复约束 | BTC 候选检索 |
| `trajectory_connectivity_state.*` | `TrajectoryConnectivityState` | 跨 session 图连通分量、首次对齐和可达性查询 | 装载历史 session 文件 |
| `pose_graph_problem.*` | `PoseGraphProblem` | 将业务约束映射为公共 `FactorGraph`，保留 ID 映射和固定变量 | 使用 GTSAM 类型 |
| `loop_pose_graph.*` | `LoopPoseGraph` | 独占活动图状态、构造快照、校验版本并提交优化结果 | 点云生命周期、优化器内部状态 |

## 处理顺序

1. 验证 `PoseConstraint` 的端点身份、测量方向、有限数值和不确定度。
2. 原子插入 constraint 与必要的 session connectivity 更新；产生新图版本。
3. 由 `PoseGraphProblem` 从一个一致图快照建立优化问题，不让 solver 迭代期间读取正在变化的容器。
4. `LoopPoseGraph` 将验证后的图快照交给 `PoseGraphSolver`，校验结果和快照版本后提交。
5. 得到 `PoseGraphResult` 后检查输入版本/请求序号；仅匹配当前图的结果可提交。
6. 通过只读更新事件通知 GlobalMapping 和前端地图修正边界，不在图锁内重切体素。

## 增量位姿图优化

`FactorGraph` 是 Atlas 的图输入模型，`PoseGraphSolver` 是唯一求解实现，内部使用 ISAM2 增量更新。当前只有这一种后端，因此不保留多实现能力协商或 solver factory；同一求解器实例要求图历史只追加。

图优化输入、选项、结果和实现边界见[因子图优化设计](../optimization.md)。

## 现有实现映射

- `backend/pose_graph/loop_pose_graph.*` 是唯一接入活动 Voxel-SLAM 的图状态 owner，负责回环试插入、快照、版本和原子提交；图优化直接调用 `FactorGraph/PoseGraphSolver`。
- 唯一求解类与图状态位于 `backend/pose_graph/`；独立同步 `PoseGraph` 包装器没有实际消费者，已删除。LoopPoseGraph 仍由 TrajectoryBuilder 持有，后续需评估是否迁移所有权，而非再增加图 facade。
- `LoopPoseGraphSnapshot` 记录 `state_version`；`CommitPoses` 校验快照版本、完整节点集合和有限/非退化位姿后一次性提交，拒绝过期结果。`Restore` 不回退版本，图状态替换会使当前版本递增。普通 `Optimize` 和回环试接收均已改为锁外求解、锁内按源版本校验并提交；候选边、优化位姿和门限结果作为一个原子提交单元。后台任务尚未改为独立的 snapshot-solve-commit 请求/结果队列。
- GTSAM solver 的同一实例只接受新增节点和因子批次；新因子可引用已有节点。要编辑既有图需创建新 solver 重建。LoopPoseGraph 的版本快照负责并发提交一致性，当前每次快照求解都创建临时 solver。
- 前端和后端之间共享的相对位姿测量使用 `common/relative_pose_constraint.hpp` 中的 `RelativePoseConstraint`；前端不再依赖 backend 的 `PoseGraph` 头文件。
- `backend/global_mapping/pgo_edges.*` 负责从多会话数据组装 PGO 节点和约束；`global_mapping/pose_graph_optimization.*` 将问题转换为公共 `FactorGraph` 并直接调用唯一增量位姿图求解器。
- 原 `optimization/voxel/pose_graph.*` 和 `loop_factor.*` 已迁出，原专用位姿图求解实现、未接入产品的 `BackendRunner`/同步 `PoseGraph` 及仅供它们使用的测试已删除。版本化提交目前只覆盖活动图位姿，尚未扩展为跨 session/map 的统一来源版本契约。
- 原始 `voxelslam.cpp::build_graph()` 和 `thd_loop_closure()` 中 ISAM2 更新是算法迁移依据；变量初始化和 factor 添加语义要以原调用链核对，不能在重命名时改变信息权重或回环门限。
