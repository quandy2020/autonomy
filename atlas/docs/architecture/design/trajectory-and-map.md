# Voxel-SLAM 会话、关键帧与地图模型

所有路径以 C++ 源码根目录 `atlas/atlas/` 为基准；为规划名，不代表已落地。原实现使用 session、scan pose、keyframe、voxel map 和重叠关键帧组；`MappingTrajectory`、`TrajectoryNode`、`LocalSubmap` 是可选建模方式，须由实际身份、生命周期和优化图需求证明。

| 文件 | 主类型 | 职责 |
|---|---|---|
| `mapping/trajectory_id.hpp` | `TrajectoryId`、`NodeId`、`SubmapId` | 强类型 ID；只有需跨模块或持久化引用时采用 |
| `mapping/mapping_trajectory.hpp/.cpp`（条件采用） | `MappingTrajectory`、`TrajectoryState` | session 稳定身份、生命周期和索引 |
| `mapping/trajectory_node.hpp/.cpp`（条件采用） | `TrajectoryNode`、`TrajectoryNode::Data` | 关键帧观测快照与可优化全局位姿分离 |
| `mapping/local_submap.hpp/.cpp`（语义成立时采用） | `LocalSubmap`、`SubmapState` | 固定成员集合、局部坐标系和图约束语义都成立时表达子地图 |
| `mapping/map_snapshot.hpp` | `MapSnapshot` | 局部/全局地图的只读版本化视图 |

若采用 `TrajectoryNode::Data`，其观测数据进入后台任务后不可变；位姿更新由图优化结果按版本提交，不回写关键帧点云。只有当 Voxel-SLAM 的局部地图块符合“固定成员+局部坐标系+图约束”的语义时，才命名为 `LocalSubmap`；否则沿用 `KeyframeGroup`、`MapBlock` 等现有算法语义。不要为套用 Cartographer 概念而额外引入没有消费者的类型。

地图领域模块的具体边界、体素数据、持久化和可视化布局见 [Mapping 模块总览](mapping-and-data.md)。
