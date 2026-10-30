# datasets

离线数据集目录通过 AutoMsgs/Autolink 话题回放，可驱动 Atlas 视觉 SLAM 或 Argus。

正文：[docs/18_Tools/08_datasets.md](../../../docs/source/18_Tools/08_datasets.md)

## 环境

```bash
# 宿主机：数据集根挂载到容器 /data
export AUTONOMY_DATASETS=/path/to/datasets
AUTONOMY_FORCE_RECREATE=1 python3 docker/run_autonomy.py

# SpaceHero 容器
cd /workspace/autonomy/src/autonomy
export LD_LIBRARY_PATH="/workspace/autonomy/install/autonomy/lib:/workspace/autonomy/build/autonomy/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="/workspace/autonomy/src/autonomy:/workspace/autonomy/build/autonomy/python:/workspace/autonomy/install/autonomy/python:/workspace/autonomy/install/autonomy/lib/python:${PYTHONPATH:-}"
```

## Atlas 示例

先启动与数据模式匹配的 Atlas profile，再运行发布器：

```bash
export PATH=/workspace/autonomy/build/autonomy/bin:$PATH
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
autolink launch start localization_atlas_euroc_stereo_inertial.launch
```

另一个终端：

```bash
cd /workspace/autonomy/src/autonomy
export LD_LIBRARY_PATH="/workspace/autonomy/install/autonomy/lib:/workspace/autonomy/build/autonomy/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="/workspace/autonomy/src/autonomy:/workspace/autonomy/build/autonomy/python:/workspace/autonomy/install/autonomy/python:/workspace/autonomy/install/autonomy/lib/python:${PYTHONPATH:-}"

/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset euroc \
  --path /data/slam/EuRoC/data/MH_01_easy \
  --atlas-sensor stereo_inertial \
  --rate 1.0
```

第三个终端启动 Autoviz：

```bash
cd /workspace/autonomy
./build/autonomy/bin/autoviz \
  --config src/autonomy/autoviz/config/atlas.autoviz
```

Atlas 默认话题：

| 输入 | 话题 |
|------|------|
| 主图 | `/camera/rgb/image_raw` |
| 右图 | `/camera/right/image_raw` |
| 深度 | `/camera/depth/image_raw` |
| IMU | `/imu` |

若平台已在 `/tf` 提供动态 `odom → base_link`，并在 `/tf_static` 提供 `base_link → camera_link`，Atlas 会根据视觉位姿发布动态 `map → odom`。最终树为 `map → odom → base_link → camera_link`；Atlas 不会再发布 `map → camera_link`，避免多父节点。`autonomy.tools.datasets` 只回放传感器数据，不伪造底盘 TF；缺少上述链路时 Atlas 会跳过 TF，但里程计、轨迹和地图点仍正常发布。

## Atlas 发布命令速查

以下命令均在上面的“另一个终端”环境中执行。先用 `info` 检查目录布局，不会发布任何话题：

```bash
/usr/bin/python3 -m autonomy.tools.datasets info \
  --dataset euroc \
  --path /data/slam/EuRoC/data/MH_01_easy \
  --atlas-sensor stereo_inertial
```

发布时，`--atlas-sensor` 必须与已启动的 Atlas launch 一致：

```bash
# EuRoC 单目；配套 localization_atlas.launch
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset euroc \
  --path /data/slam/EuRoC/data/MH_01_easy \
  --atlas-sensor monocular \
  --rate 1.0

# EuRoC 双目；配套 localization_atlas_euroc_stereo.launch
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset euroc \
  --path /data/slam/EuRoC/data/MH_01_easy \
  --atlas-sensor stereo \
  --rate 1.0

# EuRoC 单目惯性；配套 localization_atlas_euroc_mono_inertial.launch
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset euroc \
  --path /data/slam/EuRoC/data/MH_01_easy \
  --atlas-sensor monocular_inertial \
  --rate 1.0

# EuRoC 双目惯性；配套 localization_atlas_euroc_stereo_inertial.launch
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset euroc \
  --path /data/slam/EuRoC/data/MH_01_easy \
  --atlas-sensor stereo_inertial \
  --rate 1.0

# TUM RGB-D；配套 localization_atlas_tum_rgbd.launch
mkdir -p /tmp/atlas_tum_rgbd
tar -xzf /data/slam/TUM/rgbd_dataset_freiburg1_desk.tgz \
  -C /tmp/atlas_tum_rgbd
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset tum-rgbd \
  --path /tmp/atlas_tum_rgbd/rgbd_dataset_freiburg1_desk \
  --atlas-sensor rgbd \
  --rate 1.0

# TUM-VI 双目惯性；配套 localization_atlas_tum_vi_stereo_inertial.launch
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset tum-vi \
  --path /data/slam/TUM-VI/dataset-room1_512_16 \
  --atlas-sensor stereo_inertial \
  --rate 1.0

# KITTI odometry 00 双目；配套 localization_atlas_kitti_stereo.launch
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset kitti \
  --path /data/slam/kitti/odometry/sequences/00 \
  --atlas-sensor stereo \
  --rate 1.0
```

当前机器上的 TUM-VI 下载分片不完整；上面的命令格式有效，但需先准备包含完整 `mav0/` 的序列目录。快速冒烟测试可在任一 `publish` 命令末尾追加 `--start 0 --max-frames 100`。完整参数可用 `/usr/bin/python3 -m autonomy.tools.datasets publish --help` 查看。

## Spires 示例

```bash
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset spires \
  --path /data/Oxford/sequences/2024-03-12-keble-college-02 \
  --rate 1.0
```

Spires 默认发布 `/imu`、`/points`、`/camera/rgb/image_raw`、`/gt/odometry` 和 `/gt/path`。GT 优先使用 `processed/vilens-slam/gt.tum`，否则使用 `processed/trajectory/gt-tum.txt`。

## 模块

| 模块 | 职责 |
|------|------|
| `cli.py` | `info` / `publish` 命令与参数校验 |
| `publisher.py` | Spires 时序调度与 AutoMsgs 发布 |
| `visual_publisher.py` | EuRoC、TUM RGB-D/TUM-VI、KITTI 时序调度与 AutoMsgs 发布 |
| `readers/` | Spires、EuRoC/TUM-VI、TUM RGB-D、KITTI reader |
| `tum.py` | TUM 真值 |
| `pcd_io.py` / `messages.py` | PCD 与消息编码 |
