# 4. Atlas 视觉 / VIO

Atlas 是仓库内的 ORB 视觉 / 视觉惯性 SLAM。只消费相机和可选 IMU，不消费 LiDAR。标定 YAML 必须与分辨率、帧率和传感器模式一致。词表：`atlas/config/orb_vocab.fbow`。

算法与配置：[Atlas README](../atlas/README.md)、[配置](../atlas/docs/04-configuration.md)。

## AutoSim 快速验证

仿真配置：`autosim/config/atlas.yaml`。终端一：

```bash
cd /workspace/autonomy
CONFIG=src/autonomy/autosim/config/atlas.yaml \
  src/autonomy/autosim/scripts/run.sh sim
```

终端二（必须在源码根，见 [02_setup.md](02_setup.md)）：

```bash
autolink launch start localization_atlas_autosim.launch
```

终端三遥控，走有纹理、含转弯和回访的路线：

```bash
cd /workspace/autonomy
CONFIG=src/autonomy/autosim/config/atlas.yaml \
  src/autonomy/autosim/scripts/run.sh teleop
```

终端四录轨迹并评测：

```bash
cd /workspace/autonomy
source install/setup.bash
export PYTHONPATH=/workspace/autonomy/install/autonomy/python:/workspace/autonomy/install/autonomy/lib/python:${PYTHONPATH:-}
export LD_LIBRARY_PATH=/workspace/autonomy/install/autonomy/lib:/workspace/autonomy/build/autonomy/lib:${LD_LIBRARY_PATH:-}

python3 src/autonomy/autonomy/localization/atlas/examples/record_autosim_trajectories.py \
  --duration-sec 120 \
  --require-maps \
  --output-dir /workspace/autonomy/data/atlas_autosim

python3 src/autonomy/autonomy/localization/atlas/examples/evaluate_autosim.py \
  /workspace/autonomy/data/atlas_autosim/ground_truth_trajectory.tum \
  /workspace/autonomy/data/atlas_autosim/atlas_trajectory.tum \
  --output /workspace/autonomy/data/atlas_autosim/trajectory_metrics.json
```

`--require-maps` 在轨迹、稠密点云或占用栅格无有效数据时返回非零。

停止：先 `autolink launch stop localization_atlas_autosim.launch`，再停 sim / teleop。

## 公开数据集

先起与模式匹配的 launch，再 `publish`。`--atlas-sensor` 必须与 launch 里的 `--atlas_sensor` 一致。

EuRoC 单目：

```bash
autolink launch start localization_atlas.launch
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset euroc \
  --path /data/slam/EuRoC/data/MH_01_easy \
  --atlas-sensor monocular \
  --rate 1.0
```

EuRoC 双目 / 单目惯性 / 双目惯性：分别用 `localization_atlas_euroc_stereo.launch`、`localization_atlas_euroc_mono_inertial.launch`、`localization_atlas_euroc_stereo_inertial.launch`，`--atlas-sensor` 为 `stereo`、`monocular_inertial`、`stereo_inertial`。

TUM RGB-D（若 `/data` 只读，在宿主机解压后再挂载）：

```bash
autolink launch start localization_atlas_tum_rgbd.launch
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset tum-rgbd \
  --path /data/slam/TUM/rgbd_dataset_freiburg1_desk \
  --atlas-sensor rgbd \
  --rate 1.0
```

TUM-VI 512：

```bash
autolink launch start localization_atlas_tum_vi_stereo_inertial.launch
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset tum-vi \
  --path /data/slam/TUM-VI/dataset-room1_512_16 \
  --atlas-sensor stereo_inertial \
  --rate 1.0
```

KITTI 00–02（标定 `kitti_00_02.yaml`）：

```bash
autolink launch start localization_atlas_kitti_stereo.launch
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset kitti \
  --path /data/slam/kitti/odometry/sequences/00 \
  --atlas-sensor stereo \
  --rate 1.0
```

## Autoviz

```bash
cd /workspace/autonomy
./build/autonomy/bin/autoviz --config src/autonomy/autoviz/config/atlas.autoviz
```

Fixed Frame 为 `map`。预设显示主图、`/atlas/odometry`、`/atlas/trajectory`、`/atlas/map_points` 以及 `map → odom`。完整树为 `map → odom → base_link → camera_link`。底盘须在 `/tf` 提供 `odom → base_link`，标定须在 `/tf_static` 提供 `base_link → camera_link`（AutoSim 相机光心常用 `camera_optical_frame`）。数据集工具不伪造底盘 TF；缺链路时 Atlas 会跳过 TF 发布，其它输出仍可用。

## 话题

输入（按模式取子集）：

- `/camera/rgb/image_raw`
- `/camera/right/image_raw`：双目
- `/camera/depth/image_raw`：RGB-D
- `/imu`：惯性
- `/gt/odometry`：可选真值
- `/tf`、`/tf_static`

输出：

- `/atlas/odometry`
- `/atlas/trajectory`、`/atlas/optimized_trajectory`
- `/atlas/ground_truth_trajectory`
- `/atlas/map_points`、`/atlas/dense_map_points`
- `/atlas/occupancy_grid`
- `/atlas/tracking_image`、`/atlas/feature_matches`
- `/atlas/loop_closures`
- `/tf`：`map → odom`（`--atlas_publish_tf=true` 时）
