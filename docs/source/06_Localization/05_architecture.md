# 5. Localization 模块架构

`localization_main` 根据 `--localization_mode` 在三条后端里选一条，没有 AMCL 或独立 Lightning 进程。

| 模式 | 入口 |
|------|------|
| `cartographer`（默认） | `RunCartographerNode()` |
| `atlas` | Atlas 单目/双目/RGB-D 视觉 SLAM 与 VIO |
| `argus` | Argus LIO/LIVO（ESIKF） |

```text
autonomy/localization/
├── localization_main.cpp / localization_server.*
├── launch/  conf/cartographer/  atlas/config/
├── cartographer/  atlas/  argus/
```

## Cartographer

```text
localization (--localization_mode=cartographer)
    → CartographerNode
        → MapBuilderBridge（LocalTrajectoryBuilder + PoseGraph）
        → SensorBridge（LaserScan / Imu / Odom）
        → TfBridge
        → 服务：submap_query / write_state 等
```

逐步说明见 [Cartographer 指南](07_cartographer/guide.md)。

## Atlas / Argus

| 后端 | 配置 | 典型输入 | 输出 |
|------|------|----------|------|
| Atlas | `atlas/config/*.yaml` | 相机（可选 IMU） | `/atlas/odometry`、`/atlas/trajectory`、`/atlas/map_points` |
| Argus | `conf/argus/*.yaml` | IMU + 点云（+ 相机） | `/argus/odometry`、TF、点云图、占用栅格 |

Atlas：[08_atlas/guide.md](08_atlas/guide.md)。Argus：[06_argus.md](06_argus.md)。

## 与导航栈

```text
Localization → TF (map↔odom↔base) → Planning / Control
            → /map（可选）→ MapServer / Costmap static_layer
```

全栈 `global_frame`、`robot_base_frame` 与话题名要和 [Running](../04_Running/00_guide.md) 里其它域 conf 一致。
