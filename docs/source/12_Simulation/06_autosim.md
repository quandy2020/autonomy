# 6. AutoSim

用 Autolink 仿真桥给定位后端提供 IMU、点云或相机数据，而不是找 Gazebo；Atlas 使用相机/可选 IMU，Argus 使用激光+IMU（可选相机）。

`autosim`：fake / Habitat / Isaac 等后端，向栈发布 `/imu`、`/points`、相机与 TF 等。

| 需求 | 对比 |
|------|------|
| 要物理碰撞 | Gazebo [§7](07_gazebo_ros.md) |
| 要 LIO 联调 | AutoSim |
| 仅导航 costmap | Bridge 注入 `/scan` 即可 |

### 启动

```bash
CONFIG=src/autonomy/autosim/config/lightning.yaml \
  src/autonomy/autosim/scripts/run.sh sim
```

（路径随仓库布局调整。）Isaac 常用 NVIDIA 容器；**勿**用被 Kit alias 的 `python`，系统脚本用 `/usr/bin/python3`。

### 与定位联调

| 后端 | 配置提示 |
|------|----------|
| Atlas 视觉/VIO | 使用匹配仿真相机内参、分辨率、帧率和外参的自定义 YAML |
| Argus | `localization_argus.launch`；话题对齐 `lio.yaml` |

`lightning.yaml` 是 **AutoSim 场景名**，不是定位后端名称。

典型话题：`/imu`、`/points`、`/camera/rgb/image_raw`、`/tf`。

### 常见坑

| 现象 | 先查 |
|------|------|
| 定位无点云 | run.sh 是否 sim、yaml topic |
| 找 Lightning 定位 | 已移除 |

### 延伸阅读

[Atlas](../06_Localization/08_atlas/guide.md) · [Argus](../06_Localization/06_argus.md)
