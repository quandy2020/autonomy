# 模块依赖审计

本文记录 Atlas 当前 C++ 源码的静态 include 方向，区分已经满足的边界、为保留算法数据模型而存在的过渡依赖，以及下一步可迁移项。它不把“目录看起来合理”当作依赖审计结论。审计范围为 `atlas/atlas/{common,sensor,frontend,map,backend}` 下的生产头文件和实现文件；测试文件不作为生产依赖证据。

## 目标方向

```text
common <- sensor <- frontend -> backend
   ^         ^         ^          |
   |         |         |          v
   +---------+---------+--------- map
                         backend/pose_graph -> GTSAM ISAM2
```

更严格地说，箭头表示“使用者依赖提供者”。目标是：

- `common` 不依赖任何业务模块；`sensor` 只依赖 `common` 和 AutoMsgs 转换边界。
- `frontend` 使用稳定数据契约发布局部估计和关键帧；它不直接管理 backend 图实现或全局优化任务。
- `map` 管理地图结构和几何算法；它消费领域值类型，不反向依赖前端调度器或前端优化器实现。
- `backend` 管理回环约束、图状态、多会话和异步任务；`pose_graph/` 按因子类型拆分位姿图模型，求解器持有并更新 GTSAM ISAM2。

## 当前源码观察

| 使用方 | 当前依赖 | 证据 | 评估 |
|---|---|---|---|
| `frontend/trajectory_builder` | `backend/global_mapping`、`backend/loop_closure`、`backend/pose_graph` | `frontend/trajectory_builder.hpp/.cpp` | 编排器直接拥有后端对象和 PGO/回环调用；跨越 frontend/backend 的主要边界，下一步应通过窄的任务提交与结果提交契约收敛。 |
| `backend/loop_closure` | map voxel 几何数据用于三角描述子提取 | `backend/loop_closure/triangle/triangle_loop_detector.cpp` | 命名空间按目录收敛为 `atlas::backend::loop_closure` 和 `atlas::backend::loop_closure::triangle`。回环模块不再包含前端状态或修改活动地图；滑窗地图重建现由 `map/SlidingVoxelMapOperations` 执行。描述子对 map 几何类型的依赖仍需评估是否需要稳定值类型边界。 |
| `map`（源码位于 `atlas/atlas/map/`，不再设置 `voxel/` 子目录） | `keyframe.hpp` 和 `serialization/voxel_map_serializer.hpp` 仍使用前端完整状态；地图树、滑窗操作、点观测变换、点云 BA 和 HBA 使用 `common/pose_state.hpp` | `keyframe.hpp`、`serialization/voxel_map_serializer.hpp`、`sliding_voxel_tree.hpp`、`sliding_voxel_map_operations.hpp` | `PointCluster`、`PointObservation` 与 `PointCloudFactorData` 由 `atlas::map` 持有。点观测变换、滑窗树/操作及 LiDAR BA/HBA 只接收公共 `PoseState`；前端在边界创建短期位姿窗口，优化结果只提交旋转和平移，速度、偏置、重力和惯性协方差保持原样。地图不再依赖前端 odometry、惯性状态或局部 BA 实现头文件。`PointCluster` 刚体变换只接收旋转矩阵和平移向量。初始化、局部建图和轨迹编排分别使用 `atlas::frontend::initialization`、`atlas::frontend::local_mapping` 与 `atlas::frontend`；Atlas C++ 源码中已无 `vxslam` 命名空间声明或限定名。关键帧仍持有完整状态供会话保存和 backend 优化使用；序列化直接读写该状态，后续需评估是否引入公共持久化值对象。 |
| `backend/global_mapping` | frontend 状态快照、公共 pose graph 和 edge 类型；HBA 接口只使用公共 `PoseState` | `global_mapping_request.hpp`、`global_mapping_result.hpp`、`pgo_edges.hpp` | 已统一为 `atlas::backend::global_mapping`；结果身份、版本、关键帧索引、有限性和 edge 结构检查已归 request/result 契约。HBA 使用的位姿值已不再暴露完整惯性状态；request/result 快照构造和目标状态写回仍在 `TrajectoryBuilder`，TopDown PGO 也尚未表达为同一结果契约。 |
| `map/serialization` | 前端状态及点方差类型 | `serialization/voxel_map_serializer.hpp` | 现有文件格式复用运行时数据类型，短期可接受；长期需以稳定存储值对象隔离算法运行时变化。 |
| `backend/pose_graph` | `factor_graph.hpp` | `pose_graph_solver.hpp/.cpp` | 位姿图模型和 ISAM2 增量求解；GTSAM 为实现私有依赖。 |

当前未发现生产消费者的 `common::RingBuffer`、残缺的 11 字节 `map/map_options.hpp`、OpenCV YAML wrapper、动态 plugin loader 和配置目录 resolver 已清理。`ThreadSafeQueue` 是可复用通用工具，但目前没有生产消费者；全局映射以条件变量和关键帧游标驱动单 worker，不把该队列误记为当前 backend 必需组件。组件 factory/registry 保留，因为应用构建器实际使用它们注册并创建系统组件。

## 版本写入点审计

`TrajectoryBuilder` 的 map/pose 来源版本目前在以下业务状态改变后递增：历史会话发布、回环约束加入、在线 PGO、关键帧加入、HBA 局部窗口提交、submap HBA 提交、residual polish 提交及 TopDown PGO/map correction。后台请求读取 pose/map 双版本；结果提交再次比较双版本、请求 ID、session ID 和 keyframe 身份。写入版本的路径由 `map_mutex_` 保护；前端 pose correction 路径另外遵守先锁 `frontend_state_mutex_`、再锁 `map_mutex_` 的顺序。

这完成的是源码级写入点盘点，不代表多线程运行时压力验证，也不替代集中提交 API。当前阶段链由条件变量和持久游标驱动，不要求增加重复队列。

## 迁移次序

1. 已将地图滑窗操作、滑窗树、点云 BA/HBA 与点观测变换改用公共 `PoseState`；前端只在调用边界转换状态。关键帧与会话序列化仍保留完整惯性状态，因为它们承载持久化及 backend 优化需要的速度/偏置等量；后续评估公共持久化值对象，不为消除 include 而丢弃领域数据。点观测和拟合平面现已由 map 自有。
2. 让 frontend 发布不可变 `GlobalMappingRequest`，由 backend 唯一协调任务状态；TopDown PGO 也使用显式请求/结果类型。
3. 由单一提交点校验版本与身份后分别提交图状态、轨迹状态和地图重建请求；loop closure 不直接写地图。
4. 每一步先核对原 Voxel-SLAM 调用顺序和因子输入，再构建，之后在固定数据集上做数值/性能对照。

除非在相应步骤有完整替代接口和调用迁移，本审计不会据此删除公开或可复用 API。

## 命名空间规则

生产代码命名空间按源码目录表达模块职责，不使用统一的 `vxslam` 命名空间。C++17 模块声明使用完整嵌套名称，例如 `atlas::frontend::inertial`、`atlas::backend::pose_graph` 和 `atlas::map`；应用边界使用 `atlas::app`，传感器模型使用 `atlas::sensor`。当文件所在目录与类型职责不一致时，先调整文件归属或明确值类型 owner，再确定命名空间，不能通过宽泛命名空间别名掩盖目录错位。
