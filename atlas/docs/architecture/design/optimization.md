# 因子图优化

## 设计边界

`backend/pose_graph/` 包含 Atlas 位姿图因子模型与基于 GTSAM ISAM2 的增量求解器。当前产品只要求这一种优化语义，因此不保留多求解器抽象、能力协商、运行时工厂、注册表或备用实现。GTSAM 是必选依赖；缺失时配置失败，不切换到不同算法。

`LoopPoseGraph` 管理领域图状态和版本化提交；同目录的 `PoseGraphSolver` 接收经过验证的节点、先验和相对位姿约束，通过 `Update` 追加因子，再由 `CalculateEstimate` 提取结果。求解器不拥有关键帧、地图或会话状态。

## 目录与文件

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

文件名为简短的 `lower_snake_case`，类型和方法为 `UpperCamelCase`。因子类型分别定义，全部文件平铺在 `backend/pose_graph/`。调用方构造 `PoseGraphSolver`，每次只提交新增节点和因子；`UpdateResult` 返回更新状态，按需调用 `CalculateEstimate` 取得 `SolverResult`。

- `FactorGraph::IsValid` 只验证图结构与数值输入，不计算替代优化目标；`PoseNode`、`PriorFactor`、`BetweenFactor` 是 Atlas 自有输入模型，使用 Atlas 几何值类型，不泄漏第三方图对象。
- `PoseGraphSolver` 在 `.cpp` 中持有 ISAM2 状态，公开头文件只使用 Atlas 类型；按 GTSAM 的 `update` 和 `calculateEstimate` 生命周期处理新增批次。
- `PoseNode`、`PriorFactor`、`BetweenFactor` 分文件定义；`FactorGraph` 聚合这些模型，求解器配置、状态和结果与 `PoseGraphSolver` 共置。
- `SolverOptions` 仅保留最大 update 次数和重线性化阈值；`UpdateResult` 返回状态、update 次数和诊断；`SolverResult` 返回全部位姿、测量成本和诊断。
- 同一实例的 `Update` 串行执行。新因子可以引用已提交节点，新节点 ID 不得重复；编辑或删除既有历史必须创建新实例并从完整图重建。

## 求解数据流

```text
LoopPoseGraph / global pose graph assembly
        -> FactorGraph (new Atlas poses and factors)
        -> PoseGraphSolver::Update
        -> UpdateResult (status and update count)
        -> PoseGraphSolver::CalculateEstimate (when needed)
        -> SolverResult (Atlas poses, status and diagnostics)
        -> backend validates source version and commits
```

当前实现只有位姿图，因此相关文件平铺在 `backend/pose_graph/`；新增其他因子图类型时按领域职责确定目录。

## 数值与状态约定

- 位姿变量使用 `Pose3`；先验和相对位姿约束使用 GTSAM 原生因子残差。
- 噪声由旋转、平移权重或 `[rotation x, y, z, translation x, y, z]` 六维对角信息转换为标准差。零信息映射为弱约束，不作为硬约束。
- `final_cost` 使用已提交的测量因子及 GTSAM 原生目标函数计算；不纳入用于固定 gauge 的强先验。
- `maximum_update_count` 表示一次批次中 ISAM2 `update` 调用总数上限，包含新增变量和因子的首次更新。后续空更新若不再重线性化或重消元，就提前结束。
- `convergence_tolerance` 映射到 ISAM2 `relinearizeThreshold`。同一增量实例开始求解后不可改变。
- `Update` 不计算完整图成本或提取全部位姿；多批次更新后可按需调用 `CalculateEstimate`。求解器保留已提交的测量因子用于计算完整图成本，并与 ISAM2 共用因子对象，不复制整张历史图。提取全部位姿和成本仍需遍历历史。
- ISAM2 更新或结果提取发生数值异常时清除内部状态；下次调用可用完整图重新初始化。无效批次返回 `kInvalidInput`，保留已提交状态。

## 当前进度与验证边界

固定图测试比较 Atlas 适配器与原生 ISAM2 的成本和位姿；多批次测试覆盖引用已有节点、只追加因子两种更新。Backend 当前两条 PGO 路径都对版本化图快照创建临时求解器，以保留回环试插入和失败回滚语义；复用同一求解器实例的批次更新由优化库提供。
