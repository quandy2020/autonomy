# 5. Argus 激光–惯性 Voxel-SLAM

Autolink 节点 `ArgusNode` 把 `/imu`、`/points` 交给 Voxel-SLAM。`localization_argus.launch` 当前是纯 LIO：订阅图像话题但不参与估计。配置路径 `--argus_config=autonomy/localization/conf/argus/voxel_slam_default.yaml` 会映射到 `argus/config/voxel_slam_default.yaml`。

库与离线 replay：[Argus README](../argus/README.md)、[Voxel-SLAM](../argus/docs/13_voxel_slam.md)。

在线可用数据：Oxford Spires、AutoSim `autosim/config/lio.yaml`。KITTI 双目不要接这条 launch。其它 bag / MCAP 先转成文本 replay 再离线跑，不要指望任意激光包直接进 Autolink 节点。

## Oxford Spires（Autoviz）

公共环境见 [02_setup.md](02_setup.md)。数据：`/data/Oxford/sequences/`。下载见 [03_datasets.md](03_datasets.md)。

1. 启动 Argus：

```bash
autolink launch start localization_argus.launch
```

2. 确认序列后发布。等 Argus 起来再播；加 `--no-image` 与当前 LIO launch 一致：

```bash
/usr/bin/python3 -m autonomy.tools.datasets info \
  --dataset spires \
  --path /data/Oxford/sequences/2024-03-12-keble-college-02

/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset spires \
  --path /data/Oxford/sequences/2024-03-12-keble-college-02 \
  --rate 1.0 \
  --no-image
```

冒烟：`--max-frames 400`。`--rate 1.0` 为数据集实时；完整序列约三千帧。默认同时发 `/gt/odometry` 与 `/gt/path`（`--no-gt` 可关）。真值优先 `processed/vilens-slam/gt.tum`，否则 `processed/trajectory/gt-tum.txt`。

再播一次前：先停发布器，`autolink launch stop localization_argus.launch`，再 start。不要 `--loop`。

3. Autoviz：

```bash
cd /workspace/autonomy
./build/autonomy/bin/autoviz --config src/autonomy/autoviz/config/argus.autoviz
```

Fixed Frame 用 `map`。开头若干帧做 IMU / MotionInit，之后才有稳定轨迹和全局图。校园尺度把视距拉大。

| 显示 | 话题 | 含义 |
|------|------|------|
| ArgusCloudMap | `/argus/cloud_map` | 世界系全局点云（体素累积） |
| ArgusCloudRegistered | `/argus/cloud_registered` | 当前帧（已变到 map） |
| ArgusTrajectory | `/argus/trajectory` | 估计轨迹 |
| GroundTruthPath | `/gt/path` | Spires 真值 |
| ArgusOdometry | `/argus/odometry` | 当前位姿 |
| LidarScan | `/points` | 原始雷达（`lidar_link`） |

## AutoSim LIO

用 `autosim/config/lio.yaml` 发布同样的 `/imu`、`/points`，再起同一条 `localization_argus.launch`。

## 话题与 TF

输入：

- `/imu`：角速度 rad/s，加速度 m/s²
- `/points`：`sensor_msgs/PointCloud2`
- `/camera/rgb/image_raw`：launch 会订阅，当前后端不用于估计
- `/gt/odometry`、`/gt/path`：可选，仅可视化

输出：

- `/argus/odometry`
- `/argus/trajectory`
- `/argus/cloud_map`
- `/argus/cloud_registered`
- `/tf`：`map → odom`（单位）、`map → imu_link`、`imu_link → lidar_link`

## 离线文本 replay

不经过 Autolink。目录：

```text
dataset/
  imu.txt
  scans.txt
  scans/000000.txt
```

格式见 [argus/config/datasets/README.md](../argus/config/datasets/README.md)。在 Argus 构建目录：

```bash
./build/app/replay --config config/voxel_slam_default.yaml \
  --dataset /path/to/dataset --output /path/to/output
```

bag 可先用 `argus/app/tools/convert/rosbag_to_voxel_slam.py` 转成上述目录。
