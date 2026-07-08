# Localization 定位模块指南

`autonomy/localization` 是 Autonomy 的**定位与 SLAM** 子系统。当前已实现 **Cartographer** 激光 SLAM（`localization` 默认后端）与 **Atlas** 视觉 SLAM；**AMCL** 配置已预留。本文档为模块总入口，以下按 **§1–§9** 顺序组织。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 新手（激光） | [Cartographer 指南](cartographer/guide.md) → [§2 快速开始](02_quickstart.md) → [§5 架构](05_architecture.md) |
| 新手（视觉） | [Atlas 指南](atlas/guide.md) → [§2 快速开始](02_quickstart.md) → [§5 架构](05_architecture.md) |
| 算法研发 | [§9 综述](09_survey.md) → [§3 数学原理](03_math.md) → [Cartographer](cartographer/guide.md) / [Atlas](atlas/guide.md) |
| 集成调试 | [Cartographer §2](cartographer/guide.md#2-快速开始backpack-2d-数据集) → [§4 使用指南](04_usage.md) → [Cartographer §11 排错](cartographer/guide.md#11-故障排查) |

<div class="nav-costmap-banner">
  <strong>定位在导航栈中的位置</strong>
  <span class="nav-costmap-detail">传感器 → Cartographer/Atlas → TF(map↔odom↔base) → Map / Planning / Control</span>
  <span class="nav-costmap-arrow">Cartographer 发布 /map → Planning →</span>
</div>

| 编号 | 文件 | 内容 |
|------|------|------|
| 1 | [01_overview.md](01_overview.md) | 模块概览 |
| 2 | [02_quickstart.md](02_quickstart.md) | 快速开始 |
| 3 | [03_math.md](03_math.md) | 数学原理 |
| 4 | [04_usage.md](04_usage.md) | 使用指南 |
| 5 | [05_architecture.md](05_architecture.md) | 模块架构设计 |
| 6 | [atlas/guide.md](atlas/guide.md) | Atlas 视觉 SLAM |
| 7 | [07_amcl.md](07_amcl.md) | AMCL 粒子滤波定位 |
| 8 | [cartographer/guide.md](cartographer/guide.md) | Cartographer 激光 SLAM |
| 9 | [09_survey.md](09_survey.md) | 定位算法综述 |
