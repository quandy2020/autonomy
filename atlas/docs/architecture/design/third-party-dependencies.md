# 第三方依赖与最小化方案

## 目标

核心 SLAM 库以 C++17 为基线，依赖保持少且边界清晰。GTSAM 是本系统后端优化的必选库；PCL 不作为依赖，Voxel-SLAM 实际使用到的点云容器、近邻搜索与点云数据操作由 Atlas 自有模块实现。算法核心不直接依赖 ROS、AutoMsgs、可视化框架或配置解析库；数据传输、文件格式、应用和测试依赖由独立 target 引入。

## 当前依赖盘点

依据 Atlas 当前 CMake 和源码 include：

| 依赖 | 当前用途 | 目标定位 |
|---|---|---|
| Eigen3 | 几何、矩阵、协方差、特征分解，几乎贯穿核心 | 核心必需；保留 header-only target `Eigen3::Eigen` |
| Threads | 线程池与同步 | 核心必需；链接 `Threads::Threads` |
| glog | 核心中的日志宏和检查 | 当前必需，长期建议由窄的 `common/logging` 接口隔离；若不用 glog 可替换为轻量实现/标准错误输出 |
| GTSAM | ISAM2/非线性因子图后端 | 唯一优化实现且为核心必需；ISAM2 类型仅在 `pose_graph_solver.cpp` 中使用，CMake 以私有依赖链接 |
| AutoMsgs | AutoMsgs 消息转换 target | 应用/协议边界可选；核心 `atlas_voxel` 不依赖 |
| GTest | 单元测试 | 仅开发期；`BUILD_TESTING` 开启时查找，不进入安装导出依赖 |

原始 ROS Voxel-SLAM 还依赖 ROS/catkin、PCL、pcl_conversions、RViz/Qt 和 GTSAM。Atlas 不引入 ROS/PCL；从 PCL 中实际使用的能力由 Atlas 实现，GTSAM 保持必选。AutoMsgs 只在协议边界使用，RViz/Qt 仅在未来确需 GUI 展示时作为独立应用依赖。未被应用构建使用的 OpenCV YAML wrapper、配置目录 resolver 和动态 plugin loader 已删除。

## 建议的最小依赖集合

### 基础核心

```text
必需：C++17 标准库、Eigen3、Threads、GTSAM、当前阶段 glog
边界可选：AutoMsgs（消息适配/应用）、GTest（仅测试）
不依赖：ROS、PCL、RViz、Qt
```

图像、固定帧位姿和通用多传感器 collator 不属于当前 Voxel-SLAM 数据流，已从迁移工程清理；未来需要视觉前端或图像应用时，应在独立模块中定义明确的数据契约。核心日志头目前仍暴露 glog include；若要替换日志依赖，应先收窄为 Atlas 日志 API，再决定保留 glog adapter 还是删除该依赖。

### 目标 CMake target 边界

```text
atlas::core                    Eigen3 + Threads + glog + GTSAM
atlas::pose_graph_optimization  atlas::core + GTSAM（必需实现）
atlas::automsgs                 atlas::core + AutoMsgs（可选）
atlas::applications             由所选 app 组合依赖
atlas_*_test                    GTest（仅构建测试时）
```

这些是目标边界建议，不要求立刻创建空的 CMake target。只有对应源码拆分完成后再落地 target。依赖按最窄传播范围链接：公开头文件使用 Eigen/Atlas 几何类型；GTSAM 仅是求解实现的私有编译与链接依赖。

## 模块依赖规则

1. `common/`、`sensor/`、`mapping/` 不依赖 AutoMsgs、ROS、PCL、OpenCV UI 或应用代码。
2. `backend/pose_graph/` 的位姿图模型使用 Atlas 值类型；`PoseGraphSolver` 是唯一 ISAM2 封装，并保持增量状态。
3. `mapping/serialization/` 使用稳定文件格式，不序列化第三方库对象/ABI；压缩库只有在实测文件体积或 IO 成为瓶颈后再引入。
4. `mapping/visualization/` 保持纯数据；RViz、Qt、OpenGL 或 Web 库只能由展示应用 target 引入。该模块本身目前是可选提取层。
5. AutoMsgs 转换位于 `atlas_automsgs` 边界；消息 schema/protobuf 的传递依赖不向 `atlas::core` 泄漏。
6. PCL 不进入依赖图。原算法使用到的 PCL 功能由 Atlas 自有、窄职责类型实现：点云容器使用 `sensor::PointCloud`/紧凑点类型；近邻查询使用 `common/math/nearest_neighbor_index`；体素过滤与采样由 `mapping`/`sensor` 算法实现；坐标变换用 Eigen；PCD 等文件 IO 由 `mapping/serialization` 实现或通过明确格式的轻量 codec 处理。禁止为了兼容旧代码而在核心层仿造大而全的 PCL API。
7. 插件装载、命令行、日志后端、配置解析和图像解码分别按能力选用，不得因一个工具依赖就提升为全局公共依赖。

## 迁移步骤

1. 将 `glog` 调用统一到窄日志 facade，隐藏第三方宏和类型；之后再决定继续使用 glog 还是移除。
2. 将 GTSAM 设为必需依赖：`find_package(GTSAM REQUIRED)`，统一兼容上游导出的 target 名称；实现细节留在 `backend/pose_graph/pose_graph_solver.cpp`，配置缺少 GTSAM 时立即报错。当前已采用此策略，工程只实现该增量求解路径。
3. AutoMsgs、应用程序、数据转换工具、GTest 分别由 option 控制。关闭可选项后，核心 install/export 不能残留对应 package config 依赖。
4. 根 `CMakeLists.txt` 管理 Atlas 依赖、目标及测试注册；`cmake/functions.cmake` 提供源文件收集与测试构建函数，`cmake/modules/` 的文件树与 Cartographer 一致。Atlas 用 CONFIG 模式加载 Eigen3，必需依赖使用上游 CMake 包；GMock、LuaGoogle 和 Sphinx finder 保留为目录结构兼容模块，不代表 Atlas 引入这些依赖。逐项记录最低支持版本、导出 target、链接方式和许可证。不复制第三方源码进仓库，除非明确的离线构建策略要求 vendoring。

## 最终判断

短期可接受的最小核心依赖是 **Eigen3 + Threads + GTSAM + glog**。AutoMsgs 仅在消息适配/应用开启时需要；GTest 仅在测试构建时需要。原始 ROS、PCL、OpenCV、RViz/Qt 均不属于复刻核心的必要依赖，PCL 所需算法由 Atlas 自行实现。

当前 `atlas_voxel` 只查找并链接 Eigen3、Threads、glog 和必选 GTSAM；AutoMsgs 位于单独可选 target。历史依赖审计中发现的 OpenCV YAML wrapper、动态插件 loader、`${CMAKE_DL_LIBS}` 和配置文件 resolver 均已从当前核心路径清除。
