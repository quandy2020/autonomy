# 08 第三方软件

Atlas 包含由 ORB-SLAM3 迁移的代码，许可证为 GPLv3。完整许可证保留在
[COPYING](../COPYING)。

第三方依赖说明和上游许可证位置记录在
[THIRD_PARTY.md](../THIRD_PARTY.md)。主要依赖包括：

- FBow：二进制词袋与地点识别。
- g2o：图优化后端。
- 公共 math：SO(3)、SE(3) 与 Atlas Sim(3) 李群实现。
- OpenCV：图像处理与特征基础设施。
- Eigen：矩阵和几何计算。
- OpenSSL：地图文件校验支持。
- AutoMsgs 与 Autolink：autonomy 消息及运行时集成。

Atlas 不再保留 third-party 源码目录；李群功能复用 `autonomy/common/math`，
并在 Atlas 内维护轻量 Sim3 头。g2o 与 FBow 使用 SpaceHero 中 `/usr/local` 安装的系统 CMake CONFIG
包，不在 Atlas 源码树中保存副本。Google C++ 文件命名与扩展名规则仅适用
于 Atlas 自有代码，不批量改写第三方源码。
