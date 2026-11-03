# localization

`autonomy.localization` 提供两种后端：

- **Atlas**：单目、双目、RGB-D 视觉 SLAM，以及对应的视觉惯性模式。
- **Cartographer**：AutoSim 2D 激光定位与建图。

## 编译

```bash
docker exec -it SpaceHero /bin/bash
cd /workspace/autonomy
colcon build --packages-up-to autonomy
```

以下命令均在 `SpaceHero` 容器中执行。

## Atlas + AutoSim 快速验证

启动前确认没有其他定位进程，避免多个 Atlas 同时发布地图和 TF：

```bash
pgrep -af autonomy.localization
```

### 1. 启动仿真

终端一：

```bash
cd /workspace/autonomy
CONFIG=src/autonomy/autosim/config/atlas.yaml \
  src/autonomy/autosim/scripts/run.sh sim
```

### 2. 启动 Atlas

终端二必须进入源码根目录，否则 launch 中的相对配置路径无法解析：

```bash
source /workspace/autonomy/install/setup.bash
cd /workspace/autonomy/src/autonomy
export PATH=/workspace/autonomy/build/autonomy/bin:$PATH
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
autolink launch start localization_atlas_autosim.launch
```

### 3. 遥控机器人

终端三，让机器人经过有纹理、包含转弯和回访的路线：

```bash
cd /workspace/autonomy
CONFIG=src/autonomy/autosim/config/atlas.yaml \
  src/autonomy/autosim/scripts/run.sh teleop
```

### 4. 录制并评测

终端四：

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

`--require-maps` 会在轨迹、稠密点云或占用栅格无有效数据时返回非零。
结果保存在 `/workspace/autonomy/data/atlas_autosim/`。

### 5. 停止

在终端二停止 Atlas，再在 sim 和 teleop 终端按 Ctrl-C：

```bash
autolink launch stop localization_atlas_autosim.launch
```

## Atlas 模式与 launch

| launch | 模式 | 数据 |
|--------|------|------|
| `localization_atlas.launch` | `monocular` | EuRoC 单目 |
| `localization_atlas_euroc_stereo.launch` | `stereo` | EuRoC 双目 |
| `localization_atlas_euroc_mono_inertial.launch` | `monocular_inertial` | EuRoC 单目惯性 |
| `localization_atlas_euroc_stereo_inertial.launch` | `stereo_inertial` | EuRoC 双目惯性 |
| `localization_atlas_tum_rgbd.launch` | `rgbd` | TUM RGB-D Freiburg1 |
| `localization_atlas_tum_vi_stereo_inertial.launch` | `stereo_inertial` | TUM-VI 512 |
| `localization_atlas_kitti_stereo.launch` | `stereo` | KITTI 00–02 |
| `localization_atlas_autosim.launch` | `rgbd_inertial` | AutoSim |

启动其他 profile 时只需替换 launch 名称：

```bash
source /workspace/autonomy/install/setup.bash
cd /workspace/autonomy/src/autonomy
export PATH=/workspace/autonomy/build/autonomy/bin:$PATH
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch

autolink launch start localization_atlas_kitti_stereo.launch
# 停止
autolink launch stop localization_atlas_kitti_stereo.launch
```

标定文件必须与相机、分辨率和传感器模式一致。数据集发布方法见
[datasets 工具文档](../../docs/source/18_Tools/08_datasets.md)。

## Atlas 话题

输入：

- `/camera/rgb/image_raw`
- `/camera/right/image_raw`：双目模式
- `/camera/depth/image_raw`：RGB-D 模式
- `/imu`：惯性模式
- `/gt/odometry`：可选真值轨迹
- `/tf`、`/tf_static`：底盘与相机外参

主要输出：

- `/atlas/odometry`
- `/atlas/trajectory`、`/atlas/optimized_trajectory`
- `/atlas/ground_truth_trajectory`
- `/atlas/map_points`、`/atlas/dense_map_points`
- `/atlas/occupancy_grid`
- `/atlas/tracking_image`、`/atlas/feature_matches`
- `/atlas/loop_closures`
- `/tf`：Atlas 发布 `map -> odom`

## Cartographer 2D

```bash
source /workspace/autonomy/install/setup.bash
cd /workspace/autonomy/src/autonomy
mkdir -p data/maps
export PATH=/workspace/autonomy/build/autonomy/bin:$PATH
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch

autolink launch start localization_cartographer.launch
# 停止并保存 data/maps/cartographer.pbstream
autolink launch stop localization_cartographer.launch
```

默认输入为 `/scan`、`/odom`、`/tf` 和 `/tf_static`。

## 快速排查

```bash
autolink launch list
autolink --wait 3 channel list -v
pgrep -af autonomy.localization
```

- 没有输出：检查输入话题、时间戳、图像编码和标定。
- 双目或 RGB-D 无输出：检查左右图或彩色/深度图时间同步。
- VIO 不稳定：检查 IMU 单位、频率、时间戳和相机—IMU 外参。
- 地图为空：先确认跟踪成功，并让机器人移动经过有纹理区域。

## 详细文档

- [Atlas 总览](atlas/README.md)
- [Atlas 配置](atlas/docs/04-configuration.md)
- [Atlas AutoMsgs 集成](atlas/docs/05-automsgs-integration.md)
- [Atlas 测试与评测](atlas/docs/06-testing-and-benchmarking.md)
- [AutoSim Atlas 验证](../../autosim/docs/13_atlas_validation.md)
