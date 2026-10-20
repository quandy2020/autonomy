(localization-overview)=
# 1. 模块概览

| 维度 | 说明 |
|------|------|
| 职责 | 估计全局/局部位姿；可选建图 |
| 输出 | 位姿、TF `map`↔`odom`↔`base_link`、占用栅格 |
| 上游 | 激光、IMU、里程计、相机（Atlas / Argus LIVO） |
| 下游 | `map`、`planning`、`control`、可视化 |

### 1.2 子系统

| 子系统 | 传感器 | 场景 | 状态 |
|--------|--------|------|------|
| **Cartographer** | 激光 + IMU | 2D/3D 建图与定位 | **默认** |
| **Atlas** | 相机 / IMU / 激光 | VO / VIO / LIO / LIVO | — |
| **Argus** | 激光 + IMU（+ 相机） | ESIKF LIO / LIVO | [§6](06_argus.md) |

`--localization_mode=cartographer|atlas|argus`。

### 1.3 Cartographer 进程

```
scan / echoes + imu + tf → CartographerNode → submap_list, tracked_pose, TF
→（可选）occupancy_grid → /map
```

### 1.4 源码

```
autonomy/localization/
├── localization_main.cpp / localization_server.*
├── launch/ conf/{cartographer,atlas}/
├── atlas/ cartographer/ argus/
```

相关：`map`、`transform`、`driver`、可视化。
