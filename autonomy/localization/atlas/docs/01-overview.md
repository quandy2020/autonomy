# 01 Atlas 概览

Atlas 是 autonomy 工程中的视觉与视觉惯性 SLAM 实现。核心算法迁移自
ORB-SLAM3，并完成以下工程适配：

- 使用 FBow 进行词袋表达与场景识别。
- 使用 AutoMsgs 承载图像、IMU 和点云消息。
- 使用 Autolink 的 `AINFO`、`AWARN`、`AERROR` 日志接口。
- 对外提供 `autonomy::localization_atlas` CMake target。
- 支持通过有界 OpenMP 并行双目 ORB 提取。

支持的传感器模式包括单目、双目、RGB-D、单目惯性、双目惯性和 RGB-D
惯性。对外的 C++ 编排入口为 `atlas/map_builder.hpp`，运行时入口为
`atlas/system.hpp`。

Atlas 仅负责 `autonomy/localization/atlas`，不依赖也不修改 Voxel-SLAM
或 Argus。

继续阅读：

1. [架构](02-architecture.md)
2. [构建与 CMake](03-build-and-cmake.md)
3. [配置](04-configuration.md)
4. [AutoMsgs 集成](05-automsgs-integration.md)
5. [测试与评测](06-testing-and-benchmarking.md)
6. [开发规范](07-development-guide.md)
7. [第三方软件](08-third-party.md)
