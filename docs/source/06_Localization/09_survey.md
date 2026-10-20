# 9. 定位算法综述

选型前先看 [00 指南](00_guide.md) 和 [04 用法](04_usage.md)；公式背景见 [03 数学](03_math.md)。

## 三条后端各擅长什么

**Cartographer** 适合激光为主的 2D/3D 建图与纯定位，配置是 Lua，和经典 Cartographer 工作流接近。输出 TF、子图、可选 `/map`。

**Atlas** 覆盖 VO/VIO/LIO/LIVO，传感器组合灵活，库内视觉前端按 ORB-SLAM3 思路演进。进程侧用 yaml 选 mission（如 `autosim_lio.yaml` 纯 LIO）。需要多相机或和 Atlas 回环/Ceres 后端对齐时优先考虑。

**Argus** 是 FAST-LIVO2 风格的 ESIKF LIO/LIVO，激光+IMU 为主，相机可选。自带 Oxford Spires 等离线评估脚本，和 autosim 的 LIO 联调文档较完整。

三者都通过 `localization_main` 接入，**没有** 仓库内的 AMCL 粒子滤波后端。

## 怎么选

| 你的情况 | 建议 |
|----------|------|
| 室内 2D、只有激光（+ 可选 IMU） | Cartographer |
| TurtleBot3 autosim 纯 LIO（无相机） | Atlas `autosim_lio.yaml` 或 Argus `livo_autosim.yaml` |
| 多相机 / RGB-D / 异构传感器融合 | Atlas（注意 launch 默认 yaml 可能是 LIVO，别和纯 LIO 混用） |
| 离线数据集批量评测 LIO/LIVO | Argus + [Tools · 数据集](../18_Tools/08_datasets.md) |
| 已有 `.pbstream`、只要 2D 定位 | Cartographer 纯定位 launch |

Atlas 与 Argus 在「3D 激光 + IMU」上重叠：Atlas 更偏通用 SLAM 栈与 Atlas 生态；Argus 更偏 ESIKF LIVO 实现与 Spires 工具链。不确定时先在 autosim 上各跑一条，对比 TF 稳定性与 CPU。

## 数据流（共性）

```text
传感器 → Cartographer / Atlas / Argus → map→odom→base_link
                                      → 可选 OccupancyGrid / 点云图
                                      → Planning / Map
```

## 参见

[用法](04_usage.md) · [数学](03_math.md) · [Cartographer](07_cartographer/guide.md) · [Atlas](08_atlas/guide.md) · [Argus](06_argus.md) · [架构](05_architecture.md)
