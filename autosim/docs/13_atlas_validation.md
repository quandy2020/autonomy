# Atlas 验证

`config/atlas.yaml` 为 Atlas RGB-D + IMU 专用配置。Autosim 发布同步 RGB、
深度、IMU、TF、时钟和真值；Atlas 负责 `map -> odom`、轨迹、稠密点云与
占用栅格。不要同时启动其他发布 `map -> odom` 或 `/map` 的模块。

## 编译

```bash
docker exec -it SpaceHero /bin/bash
cd /workspace/autonomy
colcon build --packages-up-to autonomy
```

## 启动

终端一：

```bash
docker exec -it SpaceHero /bin/bash
cd /workspace/autonomy
CONFIG=src/autonomy/autosim/config/atlas.yaml \
  src/autonomy/autosim/scripts/run.sh sim
```

终端二：

```bash
docker exec -it SpaceHero /bin/bash
source /workspace/autonomy/install/setup.bash
cd /workspace/autonomy/src/autonomy
export PATH=/workspace/autonomy/build/autonomy/bin:$PATH
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
autolink launch start localization_atlas_autosim.launch
```

> 必须从 `/workspace/autonomy/src/autonomy` 启动；launch 内的 Atlas 配置使用
> 相对源码路径。从其他目录启动会找不到配置和词袋文件。

结束时运行：

```bash
autolink launch stop localization_atlas_autosim.launch
```

终端三使用同一份配置遥控机器人，让相机经过有纹理、包含转弯与回访的路线：

```bash
docker exec -it SpaceHero /bin/bash
cd /workspace/autonomy
CONFIG=src/autonomy/autosim/config/atlas.yaml \
  src/autonomy/autosim/scripts/run.sh teleop
```

先启动仿真和 Atlas，再启动 teleop；保持该终端具有交互式 TTY 焦点。

## 输入与输出

Atlas 输入：

- `/camera/rgb/image_raw`：`rgb8`，640x480，15 Hz
- `/camera/depth/image_raw`：`32FC1` 米，与 RGB 同时间戳
- `/imu`：200 Hz，`imu_link`
- `/gt/odometry`：`map -> base_link` 真值
- `/tf`、`/tf_static`：含 `base_link -> camera_link -> camera_optical_frame`

主要输出：

- `/atlas/odometry`、`/atlas/trajectory`
- `/atlas/optimized_trajectory`、`/atlas/ground_truth_trajectory`
- `/atlas/dense_map_points`：相对地面高度 `[-1, 1] m`
- `/atlas/occupancy_grid`
- `/atlas/tracking_image`、`/atlas/feature_matches`
- `/atlas/loop_closures`

## 验收

启动后先确认 RGB、深度和 IMU 时间持续递增，且 TF 中只有 Atlas 发布
`map -> odom`。轨迹精度使用时间关联后的 fixed-scale SE(3) ATE；至少记录
匹配位姿数、覆盖率、ATE RMSE、最大误差和轨迹长度比。单纯比较两条 Path
的原始坐标无效，因为 Atlas 地图原点由初始化帧决定。

在 Atlas 和 AutoSim 都运行后，用独立终端记录 120 秒。记录器累积原始
`/atlas/odometry` camera pose 和 `/gt/odometry` base pose；后者使用
TurtleBot URDF 的 `base_link -> camera_optical_frame` 外参转换为 camera
pose。这样既消除了转弯时的杆臂误差，也保留 Atlas epoch 跳变；累计 Path
只用于诊断，不会因 Path 清空或长度上限而只评测尾段：

```bash
docker exec -it SpaceHero /bin/bash
cd /workspace/autonomy
source install/setup.bash
export PYTHONPATH=/workspace/autonomy/install/autonomy/python:/workspace/autonomy/install/autonomy/lib/python:${PYTHONPATH:-}
export LD_LIBRARY_PATH=/workspace/autonomy/install/autonomy/lib:/workspace/autonomy/build/autonomy/lib:${LD_LIBRARY_PATH:-}
python3 src/autonomy/autonomy/localization/atlas/examples/record_autosim_trajectories.py \
  --duration-sec 120 \
  --require-maps \
  --output-dir /workspace/autonomy/data/atlas_autosim
```

也可省略 `--duration-sec`，在需要结束时按 Ctrl-C；文件仍会原子写完。若
两条 odometry 流任一缺失，或启用 `--require-maps` 后稠密点云/占用栅格无
有效数据，命令返回非零。`validation_summary.json` 会记录完整 odometry
样本数、Path 诊断、
稠密点数、栅格尺寸及已知/占用单元数。默认外参为平移
`(0.03, 0, 0.20)` m、xyzw 四元数
`(-0.5, 0.5, -0.5, 0.5)`；验证其他机器人时可用
`--base-to-camera-xyz X Y Z` 和
`--base-to-camera-quaternion QX QY QZ QW` 覆盖。

随后计算轨迹指标：

```bash
python3 src/autonomy/autonomy/localization/atlas/examples/evaluate_autosim.py \
  /workspace/autonomy/data/atlas_autosim/ground_truth_trajectory.tum \
  /workspace/autonomy/data/atlas_autosim/atlas_trajectory.tum \
  --output /workspace/autonomy/data/atlas_autosim/trajectory_metrics.json
```

Fake 后端的默认图像缺少稳定纹理，只适合消息和启动冒烟测试。轨迹与地图
验收应使用带纹理的 Habitat 或 Isaac 场景。
