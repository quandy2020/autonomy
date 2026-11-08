# Mapping 自适应体素地图与全局 HBA

> 本文描述目标设计。下列 `adaptive_voxel_map`、`adaptive_octree`、`sliding_voxel_map` 等拆分名称尚不是当前文件清单；当前实现以 `sliding_voxel_tree.hpp`、`sliding_voxel_map_operations.*`、`voxel_plane_map.*`、`point_cluster.hpp` 和 HBA 文件为准。

## 职责范围

`mapping/` 表达 Voxel-SLAM 的几何地图：按空间键定位体素，累计点簇一阶/二阶统计量，构造平面约束，支持滑动局部地图与多层体素结构。全局 HBA 的几何代价和分层优化也属于这里；HBA 的请求排队、会话收尾和结果提交由 `backend/global_mapping/` 拥有。

输入是已补偿点云、局部状态或关键帧姿态；输出是局部匹配所需的只读平面/点簇查询、map snapshot，或带状态/残差的 HBA 位姿更新。活动地图不能因全局优化耗时被锁住；优化任务只接收不可变输入。

## 目标文件树

```text
mapping/
├── cell_index.hpp                         # 体素整数坐标、分辨率层级与哈希
├── point_cluster.hpp                     # 点数、坐标和二阶矩统计
├── fitted_plane.hpp                      # 平面中心、法向、特征值和有效状态
├── adaptive_octree.hpp/.cpp              # 多分辨率体素分裂/合并和父子关系
├── adaptive_voxel_map.hpp/.cpp            # 自适应局部/全局体素地图公共操作
├── sliding_voxel_map.hpp/.cpp             # 活动窗口地图的插入、移除和边界管理
├── map_snapshot.hpp                 # 只读、版本化地图视图
├── keyframe_map_observation.hpp            # Voxel-SLAM 关键帧地图观测
├── hierarchical_bundle_adjustment.hpp/.cpp # HBA 问题执行与地图几何残差
└── internal/
    ├── sliding_voxel_tree.hpp                 # 局部地图叶节点和重切实现
    ├── bundle_adjustment_octree.hpp # 全局 HBA 的分层统计结构
    ├── plane_estimator.hpp/.cpp            # 点簇协方差特征分解与平面判据
    └── map_storage.hpp               # 索引、缓冲区和实现私有状态
```

文件只在职责或生命周期能独立测试时拆分。当前 `CellIndex` 与 hash 若十分紧密，可放进 `cell_index.hpp`；`FittedPlane` 只有多个地图算法都消费时才单独成为公共类型；小型 plane fitting 函数可留在实现文件。不要为了这棵树机械移动每个模板。

三类层级结构有不同 owner，不能互相替代：`AdaptiveOctree` 服务 `AdaptiveVoxelMap` 的空间划分，`SlidingVoxelTree` 服务 `SlidingVoxelMap` 的活动窗口和重切，`BundleAdjustmentOctree` 只组织 HBA 所需的多层统计。若实际算法复用同一种结构，应合并实现并保留清楚的适配边界，不并列维护重复树结构。

### SlidingVoxelMap 配置所有权

`SlidingVoxelMapOptions` 保存体素尺寸、层数、平面判据、每层点数下限、体素点数上限和滑窗索引映射。每个 `TrajectoryBuilder` 创建自己的配置对象，根 `SlidingVoxelTree` 持有共享只读引用并传递给子节点；插入、查询、匹配、初始化、回环重建和历史关键帧加载都显式接收该配置。初始化阶段的临时平面判据从基础配置复制，不修改其他轨迹可见的状态。地图参数不得定义为进程级可变变量。

## 类型与命名职责

| 文件 | 类型/函数 | 职责 |
|---|---|---|
| `cell_index.hpp` | `CellIndex`、`CellIndexHash` | 稳定整数体素索引，明确负坐标的 floor 规则和层级/分辨率语义 |
| `point_cluster.hpp` | `PointCluster` | 保存点数、坐标和二阶矩，支持合并/拆分/刚体变换而无需重读点 |
| `fitted_plane.hpp` | `FittedPlane` | 对外只读的平面统计和有效性状态；不直接拥有原始扫描 |
| `adaptive_octree.*` | `AdaptiveOctree` | 管理空间层级及节点分裂/合并，不拥有全局优化线程 |
| `adaptive_voxel_map.*` | `AdaptiveVoxelMap` | 目标设计中的自适应空间结构、插入/查询和只读快照创建；当前未实现 |
| `sliding_voxel_map.*` | `SlidingVoxelMap` | 维护前端有限空间窗口，移出超界体素并控制内存上限 |
| `map_snapshot.hpp` | `MapSnapshot` | 不可变、版本化、可被 backend/serializer/viewer 共享的数据引用 |
| `hierarchical_bundle_adjustment.*` | `HierarchicalBundleAdjustment` | 基于分层体素统计量建立并求解几何问题，返回独立 pose updates |
| `internal/bundle_adjustment_octree.hpp` | `BundleAdjustmentOctree` | HBA 目标函数所需分层点簇累积和邻域组织 |

`PointCluster` 只在 `count > 0` 时可求 covariance；平面建模必须对点数、特征值、法向和有限数值做检查。CellIndex 哈希必须对负坐标、体素边界、层级大小稳定。不可变 snapshot 发布后不得再修改其容器。

## HBA 的职责切分

当前 `HierarchicalBundleAdjustment` 同时暴露纯几何优化和产生会话间相对位姿约束。top-down graph edge 注入已移至 `backend/global_mapping/`，HBA 的输出也已改为 `common::SessionPoseConstraints`，由 backend 转换为其内部 `PgoEdges`。剩余职责边界如下：

- `HierarchicalBundleAdjustment` 接收固定输入快照和优化选项，运行 HBA 几何计算，并返回位姿更新和公共会话间相对位姿约束；尚未统一成一个包含状态、残差和层级统计的 `BundleAdjustmentResult`。
- backend/global mapping 创建任务、使用线程池排程、处理取消/停止、检查 map/graph version。
- 前端活动地图的重切和边缘化由 `SlidingVoxelMapOperations` 执行；线程池由 `TrajectoryBuilder` 持有并注入，任务完成后才合并复用窗口或清理体素节点。
- HBA 仍选择 session 并计算跨关键帧约束；后续若移出 session 策略，map 应仅返回局部关键帧间几何约束，由 backend 赋予 session 身份。
- `ApplyPgoEdgesToPoseGraph` 修改 PoseGraph，现由 backend 调用 PoseGraph API。
- HBA 线程数由注入的执行策略/任务执行器提供，地图算法类不独占另一套永久线程池。

## 原始代码与 Atlas 当前代码映射

| 来源 | 目标 | 保留/调整 |
|---|---|---|
| 原 `voxel_map.hpp::OctoTree` | `adaptive_octree`、`adaptive_voxel_map` 与 `internal/sliding_voxel_tree` | 保留自适应体素统计和重切行为，拆开空间结构与窗口生命周期 |
| 原 `feature_point.hpp::PVec`/`PointCluster` | `point_cluster.hpp`、`fitted_plane.hpp` | 用完整语义的点簇/平面值类型替换裸结构名，统计公式不变 |
| Atlas `sliding_voxel_tree.hpp`、`voxel_plane_map.*`、`point_cluster.hpp` | 当前滑窗和几何地图实现 | 保留已实现的树、平面查询和点簇统计；不能把零字节 `adaptive_voxel_map`/`adaptive_octree` 占位描述成可复用实现 |
| Atlas `hierarchical_bundle_adjustment.*`、`bundle_adjustment_octree.hpp` | HBA 公共算法与 internal octree | 将 PGO/session 编排方法移出 HBA，保留几何优化结果 |
| Atlas `occupancy_grid_map.*`、`kdtree_point_cloud_map.*` | 无当前实现 | 这些零字节占位及其空测试已删除；未来确有消费者时再按实际 API 设计，不预留空文件 |

## 性能和一致性

- 前端查询走只读视图/锁粒度明确的索引，插入或重切只改受影响体素；禁止每帧复制完整地图。
- 点簇尽量在局部累计矩统计，避免反复持有所有点；HBA 与回环共享云时用只读共享所有权。
- 快照记录 `MapVersion` 和坐标系/分辨率元数据；后端结果回写前做版本检查。
- 地图结构可重用连续缓冲与 reserve；体素哈希负载因子、节点分配和分裂门限由真实数据基准决定，不将阈值散落在类型方法中。
- C++17 接口不使用 `std::span`/`std::expected`；使用 const 引用、shared immutable snapshot 和明确结果 struct。
