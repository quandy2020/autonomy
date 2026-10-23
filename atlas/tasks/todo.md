# Atlas Voxel-SLAM 进度

## 本轮完成

- [x] 设置 `.clang-format` 为 Google 基础风格、C++17、80 列，并格式化 Atlas C++ 源文件。
- [x] 将 OctreeGBA 的全局 eigen/voxel 配置改为显式不可变配置，移除 GBA 路径对进程级参数的写入。
- [x] 保留不可变共享点云快照入口，避免后台 GBA 深拷贝整片点云。
- [x] GBA 配置和边生成接口使用完整语义名称，并校验 GBA 配置范围。
- [x] 线程池增加调度接口、`future` 结果/异常传递、拒绝关闭后提交、排空关闭及线程创建失败清理。
- [x] HBA 八叉树重切分改用固定线程池，跨多轮 HBA 优化复用线程；CMake 已登记线程池实现。
- [x] OctreeGBA 根、子节点使用 `unique_ptr`，节点共享不可变配置；体素坐标使用 `floor` 和 int64 范围检查。

## 仍在进行

- [ ] Cartographer 风格任务状态和任务依赖图尚未实现；当前线程池调度独立工作项。
- [ ] GBA 工作线程池仍由 `HierarchicalBundleAdjustment` 持有；`Odometry::GbaWorker` 当前按处理周期构造优化器，尚未提升为长生命周期调度器。
- [ ] SlidingOctree 仍有进程级 `voxel_size`、`max_layer`、`min_eigen_value` 状态，需要隔离。
- [ ] 前后端、地图与优化模块的依赖方向和目录职责尚未完成审计。
- [ ] 命名清理只覆盖 GBA/OctreeGBA 相关接口；OdometryConfig、map_ops 等仍有缩写字段和参数。

## 尚未验证

- [ ] 本轮未运行构建或测试；需要在依赖齐全的环境验证 C++17 编译、线程池并发行为和数据集精度。
