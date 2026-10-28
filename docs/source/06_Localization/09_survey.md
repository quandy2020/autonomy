# 9. 定位算法综述

选型前先看 [00 指南](00_guide.md) 和 [04 用法](04_usage.md)；公式背景见 [03 数学](03_math.md)。

## 三条后端各擅长什么

**Cartographer** 适合激光为主的 2D/3D 建图与纯定位，配置是 Lua，和经典 Cartographer 工作流接近。输出 TF、子图、可选 `/map`。

**Atlas** 是移植并模块化整理的 ORB-SLAM3 视觉 SLAM，支持单目、双目、RGB-D 及对应惯性模式。需要视觉回环、稀疏地图或多种相机模式时优先考虑。

**Argus** 是 FAST-LIVO2 风格的 ESIKF LIO/LIVO，激光+IMU 为主，相机可选。自带 Oxford Spires 等离线评估脚本，和 autosim 的 LIO 联调文档较完整。

三者都通过 `localization_main` 接入，**没有** 仓库内的 AMCL 粒子滤波后端。

## 怎么选

| 你的情况 | 建议 |
|----------|------|
| 室内 2D、只有激光（+ 可选 IMU） | Cartographer |
| TurtleBot3 autosim 纯 LIO（无相机） | Argus `livo_autosim.yaml` |
| 单目 / 双目 / RGB-D（可选 IMU） | Atlas（模式、标定和话题必须一致） |
| 离线数据集批量评测 LIO/LIVO | Argus + [Tools · 数据集](../18_Tools/08_datasets.md) |
| 已有 `.pbstream`、只要 2D 定位 | Cartographer 纯定位 launch |

Atlas 与 Argus 的传感器定位不同：Atlas 是视觉/VIO，Argus 是激光惯性 LIO/LIVO。按实际传感器选择，不应把 Atlas 当作 LiDAR 后端。

## 数据流（共性）

```text
激光/IMU → Cartographer / Argus → TF、地图类输出 → Planning / Map
相机/IMU → Atlas                → odometry、trajectory、map_points
```

Atlas 当前不发布 TF 或占用栅格；需要导航 TF 时由明确的下游适配层负责。

## 参见

[用法](04_usage.md) · [数学](03_math.md) · [Cartographer](07_cartographer/guide.md) · [Atlas](08_atlas/guide.md) · [Argus](06_argus.md) · [架构](05_architecture.md)
