# 运行指南

本文档介绍如何**启动、测试与验证** Autonomy 多进程导航栈，覆盖 autolink launch、Docker、板端资源实测与可选 ROS 2 集成。按 **§1–§8** 组织。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 首次运行 | [§2 快速运行](02_quickstart.md) → [§3 多进程栈](03_autonomy_process.md) |
| Docker 开发 | [§4 Docker 运行时](05_docker_runtime.md) |
| ROS 2 互操作 | [§5 ROS 2 集成](06_ros2_integration.md) |
| 板端容量 / 性能 | [§8 板端 task.launch 资源报告](09_board_task_launch_benchmark.md) |
| 排错 | [§7 故障排查](08_troubleshooting.md) |

<div class="nav-costmap-banner">
  <strong>推荐验证路径</strong>
  <span class="nav-costmap-detail">编译完成 → autolink_launch autonomy.launch → Bridge / Action 发令</span>
  <span class="nav-costmap-arrow">多进程 IPC →</span>
</div>

| 编号 | 文件 | 内容 |
|------|------|------|
| 1 | [01_overview.md](01_overview.md) | 运行概览 |
| 2 | [02_quickstart.md](02_quickstart.md) | 快速运行 |
| 3 | [03_autonomy_process.md](03_autonomy_process.md) | 多进程栈 |
| 4 | [05_docker_runtime.md](05_docker_runtime.md) | Docker 运行时 |
| 5 | [06_ros2_integration.md](06_ros2_integration.md) | ROS 2 集成 |
| 6 | [07_verification.md](07_verification.md) | 运行验证 |
| 7 | [08_troubleshooting.md](08_troubleshooting.md) | 故障排查 |
| 8 | [09_board_task_launch_benchmark.md](09_board_task_launch_benchmark.md) | 板端 task.launch CPU/内存/时延报告 |
