# 运行指南

本文档介绍如何**启动、测试与验证** Autonomy 导航栈，覆盖纯 C++ 离线运行、Docker 容器与可选 ROS 2 集成。按 **§1–§8** 组织。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 首次运行 | [§2 快速运行](02_quickstart.md) → [§4 离线导航测试](04_nav_test.md) |
| 常驻进程 | [§3 Autonomy 进程](03_autonomy_process.md) |
| Docker 开发 | [§5 Docker 运行时](05_docker_runtime.md) |
| ROS 2 互操作 | [§6 ROS 2 集成](06_ros2_integration.md) |
| 排错 | [§8 故障排查](08_troubleshooting.md) |

<div class="nav-costmap-banner">
  <strong>推荐验证路径</strong>
  <span class="nav-costmap-detail">编译完成 → autonomy_nav_test（BT 单点导航）→ 成功输出 Navigation succeeded</span>
  <span class="nav-costmap-arrow">无需 ROS 2 →</span>
</div>

| 编号 | 文件 | 内容 |
|------|------|------|
| 1 | [01_overview.md](01_overview.md) | 运行概览 |
| 2 | [02_quickstart.md](02_quickstart.md) | 快速运行 |
| 3 | [03_autonomy_process.md](03_autonomy_process.md) | Autonomy 进程 |
| 4 | [04_nav_test.md](04_nav_test.md) | 离线导航测试 |
| 5 | [05_docker_runtime.md](05_docker_runtime.md) | Docker 运行时 |
| 6 | [06_ros2_integration.md](06_ros2_integration.md) | ROS 2 集成 |
| 7 | [07_verification.md](07_verification.md) | 运行验证 |
| 8 | [08_troubleshooting.md](08_troubleshooting.md) | 故障排查 |
