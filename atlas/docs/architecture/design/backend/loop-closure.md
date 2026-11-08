# Backend 回环检测与 BTC 模块

## 模块边界

回环模块负责从当前稳定关键帧中生成描述子、检索当前会话或历史会话中的候选，并对候选执行 Voxel-SLAM 的 BTC 三角形投票和点云几何验证。它只产出“是否可信的相对测量”；不得持有 PoseGraph 的节点容器，不得直接修改关键帧全局位姿或前端体素地图。

输入必须是前端发布的稳定快照，至少包括关键帧身份、session 身份、时间戳、估计位姿、用于描述子/验证的点云视图。输出分为三个语义阶段：`LoopCandidate`（检索假设）、`LoopVerificationResult`（几何验证结论）和 pose graph 可接受的 `PoseConstraint`（已验证测量）。候选分数不能被当作约束置信度或信息矩阵。

## 推荐目录和文件名

```text
backend/loop_closure/
├── loop_candidate.hpp                  # 检索阶段的候选身份、相似度和初始变换
├── loop_verification_result.hpp       # 验证结果值类型和拒绝原因
├── loop_closure_options.hpp            # 距离、时间间隔、阈值等算法配置
├── triangle/
│   ├── triangle_descriptor.hpp          # BTC 三角形描述子值类型
│   ├── descriptor_extractor.hpp/.cpp     # 平面/角点二进制描述子
│   ├── triangle_loop_detector.hpp/.cpp   # BTC 三角形描述子的回环候选检测
│   ├── descriptor_index.hpp/.cpp        # 描述子哈希、帧映射和历史索引
│   ├── candidate_selector.hpp/.cpp      # 哈希邻域投票与候选排序
│   ├── match_verifier.hpp/.cpp          # 描述子匹配投票与相对变换估计
│   └── loop_geometry_verifier.hpp/.cpp  # 点云几何验证及退化判据
└── internal/
    └── loop_candidate_filter.hpp/.cpp   # 时间、距离、会话等通用候选门控
```

不要求上述每项都拆成独立文件：若候选选择与验证只在 `TriangleLoopDetector` 内使用且无独立状态/测试替换价值，可作为 `triangle_loop_detector.cpp` 的私有函数；若索引需要单独加载、持久化或并发查询，再保留 `DescriptorIndex`。`internal/` 不能成为低内聚工具函数的收纳目录。

## 文件职责与接口契约

| 文件/类型 | 职责和数据所有权 | 关键约束 |
|---|---|---|
| `loop_candidate.hpp` / `LoopCandidate` | query/match keyframe ID、检索分数、初始相对位姿 | 不包含已验证协方差；ID 必须包含 session 上下文或全局唯一 |
| `loop_verification_result.hpp` / `LoopVerificationResult` | 是否通过、修正变换、残差、内点率、退化指标、拒绝原因 | 输出明确失败枚举，不以 `-1` 分数同时编码多个失败原因 |
| `triangle_descriptor.hpp` / `TriangleDescriptor` | 三角形边长/方向、二进制签名、来源帧身份 | 只表达 BTC 所需信息，不携带运行时 ROS 消息 |
| `descriptor_index.*` / `DescriptorIndex` | 描述子哈希和到关键帧/平面云快照的索引 | 多会话索引需按 session 隔离并支持加载/卸载；查询期间快照保持有效 |
| `candidate_selector.*` / `CandidateSelector` | 量化边长邻域投票、二进制相似度排序、近邻帧过滤 | 检索阈值和几何验证阈值分开配置 |
| `match_verifier.*` / `MatchVerifier` | 匹配描述子投票、估计相对刚体变换 | 保留变换方向约定，使用明确的 query/match 字段名 |
| `loop_geometry_verifier.*` / `LoopGeometryVerifier` | 点云几何残差、内点比例、可观性与漂移门限检查 | 只消费输入云快照并返回结果，不改写地图/位姿 |
| `triangle_loop_detector.*` / `TriangleLoopDetector` | 组织 BTC 描述子检索并产出待验证候选 | 不直接调用 GTSAM 或写 PoseGraph |

## 处理流程

```text
KeyframeSnapshot
  -> descriptor extraction
  -> session-scoped DescriptorIndex query
  -> temporal/spatial gate
  -> triangle hash neighborhood voting
  -> rank LoopCandidate list
  -> estimate relative transform
  -> plane geometric verification
  -> LoopVerificationResult
  -> verified PoseConstraint sent to PoseGraph
```

BTC 召回阶段优先控制搜索成本和候选覆盖；几何验证阶段负责精度、退化剔除和协方差估计。两阶段分别观测耗时、候选数、通过率和失败原因，不能通过调一个相似度阈值同时替代。

## 并发与多会话

- 前端只发布只读关键帧快照；描述子提取和检索在 backend executor 上运行。
- 索引更新遵循“先构造，再发布”或受控写锁策略；查询不得看到半写入的 frame、descriptor 和 plane cloud 三者状态。
- 一个 keyframe 对多个 session 索引的引用必须带 session ID；不能用裸局部 `int frame_id` 跨会话关联。
- 任务携带来源 `MapVersion`/session revision。历史 session 被卸载或图版本变化后，过期候选不得提交为约束。
- 队列达到上限时采用明确策略（延迟、丢弃非关键查询或合并待处理关键帧）；实时前端不能等待回环完成。

## 原始源码与 Atlas 当前实现

- 原始算法：`VoxelSLAM/src/BTC.h`、`BTC.cpp`、`loop_refine.hpp`，以及 `voxelslam.cpp` 的候选过滤、几何验证和多会话检索调用。
- Atlas 当前实现：`backend/loop_closure/triangle/triangle_loop_detector.*`、`loop_geometry_verifier.*`、`descriptor_extractor.*`、`triangle_descriptor.hpp`、`triangle_plane.hpp`、`binary_descriptor.hpp`；共享候选值和检测器接口仍在 `backend/loop_closure/loop_types.hpp`。滑窗地图重建由 `map/sliding_voxel_map_operations.*` 执行。
- 目标设计去掉 `lidar/` 重复层，并拆清 detector、索引、候选和验证结果。回环检测不直接修改活动地图；轨迹状态 owner 在锁内提交位姿修正后，调用地图操作层重建滑窗地图。

当前 Atlas 的 `TriangleLoopDetector` 已持有描述子、帧 ID、位姿、平面点云和 hash table，属于现存索引 owner；目标 `TriangleLoopDetector` 可以先保留该索引状态。只有历史会话索引需要独立生命周期、并发访问或持久化时，才进一步拆出 `DescriptorIndex`。
