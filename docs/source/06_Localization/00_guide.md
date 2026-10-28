# 0. Localization 指南

## 背景

定位无输出、需要 TF 的后端没有 TF、和导航帧对不齐——选型和后端接线不对，后面 planning/control 再调也白搭。仓库里 **`localization_main` 只接三条后端**，没有独立 Lightning / AMCL 进程；仿真里的 `lightning.yaml` 只是 AutoSim 场景名。

## 思路

| 后端 | 典型场景 | 文档 |
|------|----------|------|
| **Cartographer**（默认） | 2D/3D 激光建图与定位 | [07 Cartographer](07_cartographer/guide.md) |
| **Atlas** | 相机（可选 IMU），单目/双目/RGB-D 视觉 SLAM 与 VIO | [08 Atlas](08_atlas/guide.md) |
| **Argus** | 激光+IMU（+相机）LIO/LIVO，含离线数据集 | [06 Argus](06_argus.md) |

- **`--localization_mode`** 只能是 `cartographer` / `atlas` / `argus`；写错等于没有后端。  
- 话题与外参必须和传感器或仿真一致；Cartographer/Argus 可接导航 TF，Atlas 当前发布 `/atlas/odometry`、`/atlas/trajectory` 和 `/atlas/map_points`，不发布 TF 或占用栅格。
- **默认先 Cartographer 2D** 跑通「能出 map / TF」，再换 Atlas / Argus。

离线 bag / 数据集：[Tools · bag_convert](../18_Tools/04_bag_convert.md)、[数据集](../18_Tools/08_datasets.md)。


## 步骤

### 1. 按问题选页

| 文档 | 解决什么 |
|------|----------|
| [01 概览](01_overview.md) | 三条后端对照 |
| [02 快速开始](02_quickstart.md) | Cartographer / Atlas / Argus 最短命令 |
| [03 数学](03_math.md) | 公式与模型 |
| [04 用法](04_usage.md) | 配置路径与故障排查 |
| [05 架构](05_architecture.md) | `localization_mode` 与目录 |
| [06 Argus](06_argus.md) | LIO/LIVO、话题、数据集联调 |
| [07 Cartographer](07_cartographer/guide.md) | 默认激光方案 |
| [08 Atlas](08_atlas/guide.md) | 视觉 / VIO 配置 |
| [09 综述](09_survey.md) | 怎么选后端 |

**室内 2D** → [02](02_quickstart.md) + [Cartographer](07_cartographer/guide.md)。  
**3D 激光 + IMU** → [Argus](06_argus.md) 或 Cartographer 3D。
**Oxford Spires 等离线** → [Argus](06_argus.md) + [数据集](../18_Tools/08_datasets.md)。

### 2. 动手时记住

1. Atlas 对图像/IMU 与相机标定敏感；Argus 对 IMU、点云和可选相机敏感。改传感器时要同步更新对应配置。
2. 找 Lightning 定位文档 → 已移除；改用 Atlas/Argus 或 Cartographer。

### 3. 常见坑

| 现象 | 先查 |
|------|------|
| 无 TF / 无地图 | mode；话题名；是否在播数据 |
| Atlas/Argus 无输出 | Atlas 查图像、标定和模式；Argus 查 IMU、点云与外参 |
| 和导航对不齐 | `global_frame` / `robot_base_frame` 全栈一致 |
