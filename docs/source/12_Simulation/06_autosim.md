# 6. AutoSim

用 Autolink 仿真桥给 **Atlas/Argus** 等定位喂 IMU、点云、相机，而不是找 Gazebo。

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
| Atlas LIO | `atlas/config/autosim_lio.yaml` |
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
