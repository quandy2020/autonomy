# 安装与构建指南

本文档介绍如何在宿主机、Docker 或 **嵌入式板（Firefly / aarch64）** 中安装依赖、编译 **Autonomy**。按 **§1–§9** 组织。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 首次安装（PC） | [§2 快速安装](02_quickstart.md) → [§4 依赖](04_dependencies.md) → [§6 编译](06_build.md) |
| Docker 用户 | [§5 Docker](05_docker.md) → [§6 编译](06_build.md) |
| **嵌入式板 / NFS** | [§9 嵌入式板端](09_embedded_board.md) |
| 排错 | [§8 故障排查](08_troubleshooting.md) |

<div class="nav-costmap-banner">
  <strong>推荐路径</strong>
  <span class="nav-costmap-detail">Ubuntu 22.04 → install_deps → cmake + ninja → libautonomy.so</span>
  <span class="nav-costmap-arrow">板端：NFS + --profile board →</span>
</div>

| 编号 | 文件 | 内容 |
|------|------|------|
| 1 | [01_overview.md](01_overview.md) | 安装概览 |
| 2 | [02_quickstart.md](02_quickstart.md) | 快速安装 |
| 3 | [03_system_requirements.md](03_system_requirements.md) | 系统要求 |
| 4 | [04_dependencies.md](04_dependencies.md) | 依赖安装 |
| 5 | [05_docker.md](05_docker.md) | Docker 环境 |
| 6 | [06_build.md](06_build.md) | 编译构建 |
| 7 | [07_environment.md](07_environment.md) | 环境配置 |
| 8 | [08_troubleshooting.md](08_troubleshooting.md) | 故障排查 |
| 9 | [09_embedded_board.md](09_embedded_board.md) | **嵌入式板端 NFS / 构建 / 安装** |
