# 运行时、持久化与测试

所有路径以 Atlas 工程根目录为基准；为规划名，不代表已落地。

| 文件/路径 | 主类型 | 职责 |
|---|---|---|
| `atlas/common/thread_pool.hpp/.cpp` | 现有 `ThreadPool` | 优先复用 Atlas executor；管理任务队列和 worker，不理解 SLAM 几何 |
| `atlas/backend/global_mapping/global_mapping.hpp/.cpp` | `GlobalMapping` | 接收全局优化请求、管理服务生命周期并发布结果 |
| `atlas/backend/global_mapping/global_mapping_worker.hpp/.cpp` | `GlobalMappingWorker` | 仅当现有全局 worker 需要独立资源 owner 时保留；负责 drain/cancel/join |
| `atlas/backend/global_mapping/global_mapping_request.hpp` | `GlobalMappingRequest` | 轨迹和地图版本、不可变输入快照 |
| `atlas/backend/global_mapping/global_mapping_result.hpp` | `GlobalMappingResult` | 输出状态、版本化 pose updates 和统计信息 |
| `atlas/backend/global_mapping/session_repository.hpp/.cpp` | `SessionRepository` | 会话列举、完整性校验、原子加载与保存；若当前 persistence 仍简单则不拆 |
| `atlas/map/serialization/voxel_map_serializer.hpp/.cpp` | 现有 `VoxelMapSerializer` | 地图快照的磁盘格式编解码 |
| `config/voxel_slam_default.yaml` | 当前配置与目标组织 | 唯一 YAML 文件；由 `LoadSlamSystemConfig` 加载，所有模块参数按区块归属 |
| `<module>_test.cpp` | 测试目标 | 放在被测模块附近，命名采用完整模块名，如 `trajectory_store_test.cpp` |

## 示例布局

当前优先用文档索引连接已有应用入口和唯一的 `config/voxel_slam_default.yaml`。Atlas 不再包含数据转换、评测和可视化工具；示例应说明受支持的数据输入格式、外部准备步骤、命令行和结果文件。场景数据路径由命令行提供，算法选项仍归唯一 YAML，不复制 YAML 作为 profile。没有真实独立 demo 程序时，不创建空 `examples/` 文件夹。后续新增独立运行程序时，放在独立 app/examples 目标，不复制 ROS launch 到核心算法层。

## 异步任务约定

全局任务消费 `shared_ptr<const Snapshot>`，并携带 `TrajectoryId` 和 `MapVersion`。优化完成后由 coordinator 检查版本再应用结果。关闭时停止接受新任务，按策略 drain 或 cancel，join worker，最后保存一致版本。地图优化不得在持有活动地图锁时执行耗时求解。
