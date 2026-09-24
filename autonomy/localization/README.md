# localization

进程级定位：`autonomy.localization` → `LocalizationServer`。

| 后端 / 库 | 用途 |
|-----------|------|
| **Cartographer** | 2D/3D lidar SLAM |
| **Lightning** | 独立 LIO（`lightning/`） |
| **atlas** | VO / VIO / LIO / LIVO（`atlas/`） |

```text
autonomy/localization/
├── localization_main.cpp
├── localization_server.*      # Cartographer | Lightning 门面
├── atlas/                     # VO/VIO 库（演进中）
├── cartographer/
├── lightning/
└── conf/
    ├── cartographer/
    └── lightning/
```

## 快速启动

**Cartographer**

```bash
autonomy.localization \
  --localization_mode=cartographer \
  --configuration_directory=autonomy/localization/conf/cartographer \
  --configuration_basename=autosim_2d.lua
```

或：`autolink launch start localization.launch` / `localization_cartographer.launch`

**Lightning LIO**

```bash
autonomy.localization \
  --localization_mode=lightning \
  --lightning_config=autonomy/localization/conf/lightning/autosim.yaml \
  --lightning_imu_topic=/imu \
  --lightning_lidar_topic=/points
```

或：`localization_lightning.launch`

**Atlas LIO（autosim）**

```bash
autonomy.localization \
  --localization_mode=atlas \
  --atlas_config=src/autonomy/autonomy/localization/atlas/config/autosim_lio.yaml \
  --atlas_imu_topic=/imu \
  --atlas_lidar_topic=/points
```

配置里相机关闭，外参是 TurtleBot3 的 `imu_link` → `laser_link`。`localization_atlas.launch` 用的是带相机的 `fusion_livo.yaml`，不要拿它做这次纯 LIO 检查。

## autosim 验证 Atlas LIO

工作区根目录是 `/workspace/autonomy`。三个终端都在容器里。

**1. 仿真**

```bash
cd /workspace/autonomy
CONFIG=src/autonomy/autosim/config/lightning.yaml \
  src/autonomy/autosim/scripts/run.sh sim
```

场景是 Bistro，机器人是 TurtleBot3。话题：`/imu`、`/points`、`/tf`、`/tf_static`、`/cmd_vel`。`map→odom` 由 Atlas 发，仿真只发 `odom→base_link`。

**2. 定位**

```bash
cd /workspace/autonomy
export PATH=/workspace/autonomy/build/autonomy/bin:$PATH
export LD_LIBRARY_PATH=/workspace/autonomy/build/autonomy/lib:${LD_LIBRARY_PATH:-}
export AUTOLINK_PATH=/workspace/autonomy/src/autonomy/autolink
autonomy.localization \
  --localization_mode=atlas \
  --atlas_config=src/autonomy/autonomy/localization/atlas/config/autosim_lio.yaml \
  --atlas_imu_topic=/imu \
  --atlas_lidar_topic=/points
```

日志里应出现 `AtlasNode started mode=lio mission=mapping`。词袋文件打不开可以忽略，这次没有开相机。开头可能有一次 `sync package failed`，随后每帧应持续打印 `LIO get cloud`，匹配点数大约四千。静止时位置在原点附近，重力模长约 9.81。

**3. 向前开一段**

另开终端，环境与仿真脚本相同（`PYTHONPATH` 含 autosim、autolink Python 和 automsgs）。向 `/cmd_vel` 发 `TwistStamped`，`linear.x = 0.35`，持续约 4 秒，频率高于 2 Hz（看门狗 0.5 秒）。然后发零速度。

**4. 对照输出**

| 话题 | 期望 |
|------|------|
| `/atlas/odometry` | 前进后 x 约 1.3 m，y、z 接近 0，偏航接近 0 |
| `/atlas/cloud_registered` | 每帧数千点，坐标系 `map` |
| `/atlas/occupancy` | 非空栅格，边长大约 200 |
| `/atlas/cloud_map` | 约 1 秒一次，带高度颜色 |
| `/tf` | `map→odom`，没有轮式里程计时再补 `odom→base_link` |

停进程：结束定位进程和 `python3 -m autosim`。

## 常用 flags

| Flag | 说明 |
|------|------|
| `--localization_mode` | `cartographer` \| `lightning` \| `atlas` |
| Cartographer | `--configuration_directory` / `--configuration_basename` / `--save_state_filename` |
| Lightning | `--lightning_config` / `--lightning_imu_topic` / `--lightning_lidar_topic` |
| Atlas | `--atlas_config` / `--atlas_imu_topic` / `--atlas_lidar_topic` / `--atlas_image_topic` |

atlas 结构见 [atlas/README.md](atlas/README.md)。
