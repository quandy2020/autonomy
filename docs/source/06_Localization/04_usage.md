(localization-usage)=
# 4. 使用指南

三条后端共用 `autonomy.localization`，通过 `--localization_mode` 切换。Cartographer 使用 Lua 配置；Atlas 和 Argus 使用各自的 YAML 与话题参数。

## Cartographer

完整步骤：[07_cartographer/guide.md](07_cartographer/guide.md)

| 项 | 值 |
|----|-----|
| 配置 | `autonomy/localization/conf/cartographer/` |
| Launch | `localization.launch`、`cartographer_2d.launch` 等 |

```bash
autonomy.localization \
  --localization_mode=cartographer \
  --configuration_directory=autonomy/localization/conf/cartographer \
  --configuration_basename=autosim_2d.lua \
  --save_state_filename=data/maps/cartographer.pbstream
```

纯定位：`--load_state_filename` + `pure_localization_trimmer`（见 Cartographer 指南）。

## Atlas

Atlas 是 ORB 视觉 SLAM / VIO 后端，只消费相机和可选 IMU，不消费 LiDAR。完整说明见 [08_atlas/guide.md](08_atlas/guide.md)。

```bash
cd /workspace/autonomy/src/autonomy
export PATH=/workspace/autonomy/build/autonomy/bin:/usr/local/bin:/usr/bin:/bin
export AUTOLINK_LAUNCH_PATH=/workspace/autonomy/src/autonomy/autonomy/localization/launch

autolink launch start localization_atlas.launch
autolink launch list
autolink launch stop localization_atlas.launch
```

launch 默认使用 `monocular`、`euroc_mono.yaml`、`orb_vocab.fbow` 和 `/camera/rgb/image_raw`。切换双目、RGB-D 或惯性模式时，必须同步修改 `atlas_sensor`、相机/IMU 标定 YAML 和对应输入话题。

Atlas 当前只发布：

- `/atlas/odometry`
- `/atlas/trajectory`
- `/atlas/map_points`

它不发布 TF、占用栅格、registered cloud 或稠密 cloud map。

## Argus

见 [06_argus.md](06_argus.md)。离线 bag：[Tools · 数据集](../18_Tools/08_datasets.md)。

## 与导航栈

不同后端的输出能力不同：

| 后端 | 主要输出 |
|------|----------|
| Cartographer | TF、子图及可选占用栅格 |
| Atlas | 视觉里程计、轨迹、稀疏地图点 |
| Argus | LIO/LIVO 位姿、TF 和地图类输出 |

Atlas 若要接入依赖 `map→odom→base_link` 的导航栈，需要显式的下游适配层，不能假定 Atlas 自己发布 TF。

| 场景 | 后端 |
|------|------|
| 激光 2D 建图/定位 | Cartographer |
| 3D 激光 + IMU | Argus 或 Cartographer 3D |
| 单目 / 双目 / RGB-D（可选 IMU） | Atlas |

## 故障排查

| 现象 | 排查 |
|------|------|
| Cartographer 无 `/map` | Lua `publish_occupancy_grid`；是否使用正确激光话题 |
| Atlas 无输出 | `atlas_sensor`、图像/IMU 话题、标定、词表路径是否一致 |
| Argus 无输出 | IMU/点云话题、外参与 LIVO 图像输入 |
| 需要 TF 但链路中断 | 先确认所选后端是否发布 TF，再检查 frame 配置 |
| 找 AMCL | 当前未合入；2D 激光使用 Cartographer 纯定位或 SLAM |

## 常用 flags

| 后端 | flags |
|------|-------|
| 通用 | `--localization_mode` |
| Cartographer | `--configuration_directory` / `--configuration_basename` / `--save_state_filename` |
| Atlas | `--atlas_config` / `--atlas_vocabulary` / `--atlas_sensor` / `--atlas_image_topic` / `--atlas_right_image_topic` / `--atlas_depth_image_topic` / `--atlas_imu_topic` |
| Argus | `--argus_config` / `--argus_imu_topic` / `--argus_lidar_topic` / `--argus_image_topic` |
