# Atlas Voxel-SLAM 进度

## 本轮完成

- [x] 设置 `.clang-format` 为 Google 基础风格、C++17、80 列，并格式化 Atlas C++ 源文件。
- [x] 将 OctreeGBA 的全局 eigen/voxel 配置改为显式不可变配置，移除 GBA 路径对进程级参数的写入。
- [x] 保留不可变共享点云快照入口，避免后台 GBA 深拷贝整片点云。
- [x] GBA 配置和边生成接口使用完整语义名称，并校验 GBA 配置范围。
- [x] 线程池增加调度接口、`future` 结果/异常传递、拒绝关闭后提交、排空关闭及线程创建失败清理。
- [x] HBA 八叉树重切分改用固定线程池，跨多轮 HBA 优化复用线程；CMake 已登记线程池实现。
- [x] OctreeGBA 根、子节点使用 `unique_ptr`，节点共享不可变配置；体素坐标使用 `floor` 和 int64 范围检查。
- [x] 修复运动初始化与局部 BA 因子集合的生命周期，确保八叉树索引在首次及后续滑窗边缘化时有效。
- [x] 应用层补齐点云逐点时间戳配置，采用正向语义名称并覆盖 YAML 往返和运动端到端测试。
- [x] 实现 Cartographer 风格的任务状态机与依赖图，覆盖依赖顺序、等待和失败释放语义。
- [x] 局部地图重切分及边缘化复用 `Odometry` 长生命周期线程池，不再按帧创建裸线程。

## 仍在进行

- [ ] GBA 工作线程池仍由 `HierarchicalBundleAdjustment` 持有；`Odometry::GbaWorker` 当前按处理周期构造优化器，尚未提升为长生命周期调度器。
- [ ] SlidingOctree 仍有进程级 `voxel_size`、`max_layer`、`min_eigen_value` 状态，需要隔离。
- [ ] 前后端、地图与优化模块的依赖方向和目录职责尚未完成审计。
- [ ] 命名清理已覆盖 GBA/OctreeGBA 和地图并行操作接口；`OdometryConfig` 等仍有历史缩写字段和参数。

## 已验证

- [x] `SpaceHero` 中完整 C++17 工程构建通过，CTest 37/37 通过。
- [x] `ATLAS_BUILD_APPLICATIONS=OFF`、`ATLAS_BUILD_MESSAGE_ADAPTERS=OFF`、`BUILD_TESTING=OFF` 的核心库独立构建通过。

## 尚未验证

- [ ] 使用与原版相同的真实数据集进行轨迹精度、回环、多会话和性能对比。
