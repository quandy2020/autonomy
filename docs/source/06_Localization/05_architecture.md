# 5. Localization 模块架构

### 5.1 进程入口

`localization_main` 按 `--localization_mode` 选择后端：

| 模式 | 入口 | 状态 |
|------|------|------|
| `cartographer`（默认） | `RunCartographerNode()` | — |
| `atlas` | Atlas VO/VIO/LIO/LIVO | — |
| `argus` | Argus LIO/LIVO | — |

```
autonomy/localization/
├── localization_main.cpp / localization_server.*
├── launch/ conf/{cartographer,atlas}/
├── cartographer/ atlas/ argus/
```

### 5.2 Cartographer

```
localization (--localization_mode=cartographer)
│
▼
CartographerNode
├── MapBuilderBridge → LocalTrajectoryBuilder + PoseGraph
├── SensorBridge → LaserScan / Imu / Odom
├── TfBridge
└── 服务：submap_query / write_state 等
```

详：[07_cartographer/guide.md](07_cartographer/guide.md)。

### 5.3 Atlas / Argus

| 后端 | 配置 | 输入 | 输出要点 |
|------|------|------|----------|
| Atlas | `atlas/config/*.yaml` | 相机/IMU/激光 | `/atlas/odometry`、TF、occupancy 等 |
| Argus | `argus/config/` | 激光+IMU(+相机) | LIO/LIVO 里程计 |

Atlas：[08_atlas/guide.md](08_atlas/guide.md)。Argus：[§6](06_argus.md)。

### 5.4 与导航栈

```
Localization → TF (map↔odom↔base) → Planning / Control
→ /map（可选）→ MapServer / Costmap static_layer
```
