# Localization 定位模块指南

`autonomy/localization` 是 Autonomy 的**定位与 SLAM** 子系统。当前代码库已实现 **Atlas** 视觉 SLAM（单目 / 双目 / RGB-D）；**AMCL** 与 **Cartographer** 配置已预留，算法实现尚在集成中。本文档为模块总入口，以下按 **§1–§8** 顺序组织。

**推荐阅读路径**

| 角色 | 建议顺序 |
|------|----------|
| 新手 | [§1 概览](01_overview.md) → [§2 快速开始](02_quickstart.md) → [§5 架构](05_architecture.md) → [§4 使用指南](04_usage.md) |
| 算法研发 | [§8 综述](09_survey.md) → [§3 数学原理](03_math.md) → [§6 Atlas](06_atlas.md) |
| 集成调试 | [§2 快速开始](02_quickstart.md) → [§4 使用指南](04_usage.md) → [§4.7 排错](04_usage.md#47-故障排查) |

<div class="nav-costmap-banner">
  <strong>定位在导航栈中的位置</strong>
  <span class="nav-costmap-detail">传感器 → Localization → TF(map↔odom↔base) → Map / Planning / Control</span>
  <span class="nav-costmap-arrow">Atlas 可同步建图 → MapServer →</span>
</div>

| 编号 | 文件 | 内容 |
|------|------|------|
| 1 | [01_overview.md](01_overview.md) | 模块概览 |
| 2 | [02_quickstart.md](02_quickstart.md) | 快速开始 |
| 3 | [03_math.md](03_math.md) | 数学原理 |
| 4 | [04_usage.md](04_usage.md) | 使用指南 |
| 5 | [05_architecture.md](05_architecture.md) | 模块架构设计 |
| 6 | [06_atlas.md](06_atlas.md) | Atlas 视觉 SLAM |
| 7 | [07_amcl.md](07_amcl.md) | AMCL 粒子滤波定位 |
| 8 | [09_survey.md](09_survey.md) | 定位算法综述 |
