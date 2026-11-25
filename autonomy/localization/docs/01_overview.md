# 1. 总览与选型

二进制 `autonomy.localization` 是 Autolink 上的定位进程。后端由 `--localization_mode` 决定：

| 模式 | 传感器 | 用途 |
|------|--------|------|
| `atlas` | 单目 / 双目 / RGB-D，可选 IMU | 视觉 SLAM、视觉惯性 |
| `argus` | IMU + 3D 点云 | 激光–惯性 Voxel-SLAM（当前 launch 为纯 LIO） |
| `cartographer` | 2D `/scan` + `/odom` | AutoSim / Habitat 平面建图与定位 |

不要把 KITTI 双目接到 Argus：KITTI odometry 灰度序列没有配套 IMU，走 Atlas `localization_atlas_kitti_stereo.launch`。公开 3D 激光在线回放用 Oxford Spires + Argus。

## 进程与 launch

| launch | 模式 | 配置 |
|--------|------|------|
| `localization_atlas.launch` | Atlas `monocular` | `atlas/config/euroc_mono.yaml` |
| `localization_atlas_euroc_stereo.launch` | `stereo` | EuRoC 双目 |
| `localization_atlas_euroc_mono_inertial.launch` | `monocular_inertial` | EuRoC 单目惯性 |
| `localization_atlas_euroc_stereo_inertial.launch` | `stereo_inertial` | EuRoC 双目惯性 |
| `localization_atlas_tum_rgbd.launch` | `rgbd` | TUM RGB-D Freiburg1 |
| `localization_atlas_tum_vi_stereo_inertial.launch` | `stereo_inertial` | TUM-VI 512 |
| `localization_atlas_kitti_stereo.launch` | `stereo` | KITTI 00–02 |
| `localization_atlas_autosim.launch` | `rgbd_inertial` | AutoSim `atlas.yaml` |
| `localization_argus.launch` | Argus | `conf/argus/voxel_slam_default.yaml` → `argus/config/` |
| `localization_cartographer.launch` | Cartographer | `conf/cartographer/autosim_2d.lua` |

相对路径（`--atlas_config`、`--argus_config`、词表）相对**源码包根**解析。必须在 `/workspace/autonomy/src/autonomy` 启动 launch。

## 工作流顺序

1. 编译 `autonomy`。
2. 准备数据：下载脚本写入 `$AUTONOMY_DATASETS` / `/data`，或启动 AutoSim。
3. `autolink launch start <profile>.launch`，确认进程已起来。
4. 再播数据集或遥控仿真。先播后起后端会丢开头若干帧，且 Atlas 词表加载期间不补发。
5. Autoviz 看话题。
6. 停发布器，再 `autolink launch stop`。再次播放前必须重启后端，避免时间戳回绕。
