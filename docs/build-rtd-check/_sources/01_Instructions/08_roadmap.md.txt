# 8. 版本与路线

## 8.1 当前版本

| 项目 | 值 |
|------|-----|
| 版本号 | **0.1.1**（见 `version.json`） |
| 语言标准 | C++17 |
| 构建系统 | CMake ≥ 3.20 |
| 许可证 | Apache 2.0 |

## 8.2 实现状态摘要

| 模块 | 完成度 | 说明 |
|------|--------|------|
| autolink | ✅ 高 | 通信运行时可用 |
| map / costmap | ✅ 高 | 2D 代价地图完整 |
| planning | ✅ 高 | 三种规划器 + 后处理 |
| localization (Atlas) | ✅ 高 | 视觉 SLAM 可用 |
| control | ⏳ 中 | Checker/Smoother 就绪，主循环待完成 |
| navigator | ⏳ 中 | BT XML/配置就绪，引擎与插件待恢复 |
| bridge (gRPC) | ⏳ 低 | 部分 stub |
| perception / prediction | ⏳ 低 | 扩展占位 |

## 8.3 近期方向

1. **Control**：完成 `ControllerServer` FollowPath 循环与 MPPI / Graceful 等控制器插件
2. **Navigator**：恢复 BtEngine、52 个 BT 插件与 `BtNavigator`，接通 `Autonomy` BT 模式
3. **Localization**：AMCL / Cartographer 配置落地为 C++ 实现
4. **Bridge**：完善 gRPC Navigator 客户端与服务端
5. **文档**：各模块 §1–§9 持续补全（与代码同步）

## 8.4 变更记录

发布说明见仓库根目录 [`CHANGELOG.rst`](https://github.com/quandy2020/autonomy/blob/main/CHANGELOG.rst)。

## 8.5 致谢

Autonomy 借鉴并受益于开源社区诸多项目，包括但不限于：

- [Navigation2](https://github.com/ros-navigation/navigation2)
- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
- [Cartographer](https://github.com/cartographer-project/cartographer)
- [Autoware Universe](https://github.com/autowarefoundation/autoware_universe)
- [ROS 2](https://github.com/ros2)

## 8.6 参与贡献

1. Fork 仓库并创建特性分支
2. 遵循现有代码风格（`scripts/format.py`）
3. 提交 Pull Request 并附测试说明
4. 重大变更请同步更新对应模块文档

## 8.7 相关文档

- [§1 项目概览](01_overview.md)
- [§7 文档导读](07_documentation_guide.md)
