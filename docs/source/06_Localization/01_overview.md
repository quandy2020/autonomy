(localization-overview)=
# 1. 模块概览

定位域估计机器人在地图中的位姿（建图可选）。Cartographer/Argus 可向导航栈提供 TF 与地图类输出；Atlas 当前提供视觉里程计、轨迹和稀疏地图点。输入按后端分别来自激光、IMU、里程计或相机。

用一张职责表建立索引，细节跳到专题页，不在概览里展开命令序列。

## 三条后端

| 后端 | 传感器 | 典型场景 | 文档 |
|------|--------|----------|------|
| **Cartographer**（默认） | 激光 + IMU | 2D/3D 建图、纯定位 | [07](07_cartographer/guide.md) |
| **Atlas** | 相机（可选 IMU） | 单目 / 双目 / RGB-D 视觉 SLAM 与 VIO | [08](08_atlas/guide.md) |
| **Argus** | 激光 + IMU（+ 相机） | ESIKF LIO / LIVO | [06](06_argus.md) |

进程开关：`--localization_mode=cartographer|atlas|argus`。没有 AMCL 后端。选型见 [09](09_survey.md)。

## Cartographer 数据流（默认）

```text
scan / echoes + imu + tf
    → CartographerNode
    → submap_list · tracked_pose · TF
    →（可选）occupancy_grid → /map
```

## 源码位置

```text
autonomy/localization/
├── localization_main.cpp / localization_server.*
├── launch/  conf/cartographer/  atlas/config/
├── cartographer/  atlas/  argus/
```

话题名、`map`/`base_link` 与 map、planning、control conf 对齐。最短命令：[02 快速开始](02_quickstart.md)。
