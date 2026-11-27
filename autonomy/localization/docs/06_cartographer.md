# 6. Cartographer 2D

以下命令在 SpaceHero 容器内执行。同一时刻只起一个定位后端，必须在 `/workspace/autonomy/src/autonomy` 启动 launch。

| launch | 配置 | 数据 |
|--------|------|------|
| `localization_cartographer.launch` | `backpack_2d.lua` | Museum record：`/echoes_1` + `/imu` |
| `localization_cartographer_autosim.launch` | `autosim_2d.lua` | AutoSim `/scan` + `/odom` |

Lua 细节：[Cartographer 指南](../../../docs/source/06_Localization/07_cartographer/guide.md)。

## Deutsches Museum record

转换好的文件：

`/workspace/autonomy/data/records/cartographer/cartographer_paper_deutsches_museum.record`

准备三个容器终端，**每个终端先执行一次**：

```bash
source /workspace/autonomy/install/setup.bash
cd /workspace/autonomy/src/autonomy
mkdir -p data/maps
export PATH=/workspace/autonomy/build/autonomy/bin:$PATH
export LD_LIBRARY_PATH=/workspace/autonomy/build/autonomy/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export AUTOLINK_PATH=$PWD/autolink/autolink
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
```

必须同时设置 `PATH` 和 `LD_LIBRARY_PATH`：前者选择 build 程序，后者选择同一版本的 build 动态库。只设置 `PATH` 会加载 `install/autonomy/lib` 的旧库，可能造成 TF 时间戳和位姿异常。设置环境后需重新启动旧进程才会生效。

1. 终端一启动 Cartographer（终端保持运行）：

```bash
autolink launch start localization_cartographer.launch
```

启动日志应包含 `Cartographer backend started`，配置路径应为 `/workspace/autonomy/src/autonomy/autonomy/localization/conf/cartographer/backpack_2d.lua`。从工作空间根目录启动时，相对配置路径可能回退到 install 中的旧配置。

2. 终端二等定位进程起来后再播 record（`-a` 播全部频道，不要 `--loop`）：

```bash
autolink recorder play -a -r 1.0 -f \
  /workspace/autonomy/data/records/cartographer/cartographer_paper_deutsches_museum.record
```

冒烟可把 `-r 1.0` 改为 `-r 5` 加快。完整一条约 32 分钟实时。重新回放前先停止播放器和定位进程，等旧进程完全退出，再按步骤 1、2 启动，避免时间戳回绕。

3. 终端三启动 Autoviz，Fixed Frame 为 `map`：

```bash
cd /workspace/autonomy
./build/autonomy/bin/autoviz \
  --config src/autonomy/autoviz/config/cartographer.autoviz
```

| 显示 | 话题 |
|------|------|
| OccupancyMap | `/map` |
| TF | `map → odom → base_link → imu_link / horizontal_laser_link` |
| TrackedPose (`PoseStamped`) | `/tracked_pose` |
| Trajectory (`Path`) | `/trajectory` |

开头若干扫描会累积（`num_accumulated_range_data = 10`），之后才出 `/map`。博物馆尺度把视距拉到几十米。

OccupancyMap 使用 RViz ROS 的 `map` 配色和默认 Alpha `0.7`：空闲为白色、占用为黑色、未知为灰绿色 `#708986`。该灰绿色就是 RViz 的未知栅格颜色；需要查看不透明的原始调色板时，将 Alpha 改为 `1.0`。参考 [RViz 官方 MapDisplay](https://github.com/ros-visualization/rviz/blob/noetic-devel/src/rviz/default_plugin/map_display.cpp)。

`/map` 保留 Cartographer 的占用概率和原始墙体宽度，不做黑白二值化、墙体膨胀或未知边界补墙。中间概率显示为灰度；修复前运行的节点需要重新启动才能使用此逻辑。

`/trajectory` 发布位姿图中全部优化后的 SLAM 节点，保留起点及历史路段，并随回环优化更新；不再使用只保留最近 8000 个位姿的滑动缓存。修复前启动的 Cartographer 需要重新启动。Autoviz 的 `Trajectory` 保持 `Buffer Length = 1`、`Prune Behind Robot = false`、`Hide When Arrived = false` 即可显示整条轨迹；这里的 Buffer Length 是消息帧数，不是轨迹点数。

需要查看独立子图、回环和约束时：

1. 编译完成后，先停止回放，再重新启动 Cartographer 和 Autoviz，使用上面的 `cartographer.autoviz` 配置。
2. 取消勾选 `OccupancyMap`，勾选 `Submaps`，Channel 保持 `/submap_list`。等状态中的纹理数量逐渐补齐；后打开的窗口也能接收已有子图。
3. 展开 `Submaps` 中的子图列表：`Trajectory 0` 等分组下，每一行是“子图编号.版本号”（例如 `7.180`）。勾选或取消勾选可独立显示／隐藏，轨迹开关控制整组；双击子图行只看该子图，勾选 `All Enabled` 恢复全部。也可以先取消 `All Enabled`，再勾选需要的几张子图。`Submap IDs` 和 `Submap Pose Axes` 可单独开关。
4. 二维子图使用 `High Resolution`；三维子图可切换 `Low Resolution`。`Alpha` 调整纹理透明度。只看位姿时关闭 `Submap Textures`，再打开 `OccupancyMap`。
5. 勾选 `Constraints`（`/constraint_list`），按需开关六类连线：`Intra constraints` 是局部约束，按子图着色；`Inter constraints, same trajectory` 是同轨迹回环约束（黄）；`Inter constraints, different trajectories` 是跨轨迹约束（橙）。对应的 `residuals` 显示优化后误差：局部为红，回环及跨轨迹为青。

按高度淡出：`Tracking Frame` 默认 `base_link`，`Fade-out Distance` 默认 `1` 米。子图原点与跟踪坐标系原点在地图坐标系中的高度差不超过 1 米时保持原透明度，2 米时透明度减半，3 米及以上时纹理完全透明；最终透明度还会乘以 `Alpha`。坐标轴和编号保持可见。将距离设为 `0` 可关闭淡出；跟踪 TF 缺失时显示警告，纹理保持原透明度。修改后只需重启 Autoviz 加载新版本。

子图编号、坐标轴、纹理方向、约束配色、2.5 厘米线宽和残差端点参照 Cartographer ROS/RViz。纹理通过 `/submap_list` 分批发布并按版本缓存，Autoviz 不调用 RPC；回环优化只改位姿时不会重新解压纹理。`Constraints` 默认关闭，勾选后才发布约束数据。独立 Autoviz 构建未包含 Cartographer 消息时不注册这两个显示项。

此配置的 Orbit 视角跟随 `base_link`，Global Fixed Frame 仍为 `map`。已经打开的窗口可在右侧 Views 中把 Target Frame 设为 `base_link`，再将 Focal Point 的 X/Y/Z 都设为 `0`、Distance 设为 `40`、Pitch 设为 `1.37`，让相机围绕当前机器人观察。激光保持真实三维姿态，不强行压到地图的 Z=0 平面；状态为 Ok 但看不到时，先检查视野是否仍停在原点。

若显示 `TF missing: horizontal_laser_link -> map`，另开一个终端执行公共环境后检查：

```bash
timeout 15 autolink channel echo /tf_static --once
timeout 15 autolink channel echo /tf --once
```

`/tf_static` 应含 `base_link → imu_link / horizontal_laser_link / vertical_laser_link`；`/tf` 应含动态的 `map → odom → base_link`。Museum 回放的动态 TF 时间戳应在 2015 年，与传感器数据一致，不能外推到当前年份。仅看到 `/tf_static` 的发布日志，还不能证明整条变换链可用。

4. 停止：Ctrl-C 停播放器，再：

```bash
autolink launch stop localization_cartographer.launch
```

状态写到启动目录下的 `data/maps/cartographer.pbstream`，即 `/workspace/autonomy/src/autonomy/data/maps/cartographer.pbstream`。

## AutoSim / Habitat

```bash
autolink launch start localization_cartographer_autosim.launch
```

仿真用带 `/scan` 的 slam 配置。或 `scripts/start_cartographer.sh`（现在起 backpack launch）。

## 轮速 + IMU：EKF 与参数调优

以下命令在 SpaceHero 容器中执行，工作目录为 `/workspace/autonomy/src/autonomy`，公共环境同上。

1. 修改 `autonomy/localization/conf/cartographer/wheel_imu_ekf.yaml` 中的轮速、IMU 频道和坐标系；修改同目录 `robot_2d_ekf.lua` 中的激光频道、量程。轮速输入必须是尚未融合 IMU 的编码器速度，`header.frame_id = odom_frame`、`child_frame_id = base_frame`。已有 IMU 到 base、激光到 base 的真实标定 TF 由驱动/URDF 发布。
2. 关闭轮速驱动的 `odom → base_link` TF；先停止旧 Cartographer，再启动整套链路：

```bash
autolink launch start localization_cartographer_ekf.launch
```

EKF 发布 `/odometry/filtered` 和 `odom → base_link`；Cartographer 发布 `map → odom`。停止时执行 `autolink launch stop localization_cartographer_ekf.launch`。任一进程退出时 launch 会停止整组，避免 EKF 单独重启造成里程计原点跳变。

3. 检查融合输出，再开始机器人建图：

```bash
timeout 10 autolink channel echo /odometry/filtered --once
```

应包含传感器时间戳、`odom`/`base_link`、位姿和两组 36 项协方差。超过 `wheel_timeout` 没有轮速时停止继续推算；轮速恢复后保留当前位姿，增大不确定性，不补造失联期间的运动。

这是平面六状态 EKF（x/y/yaw/vx/vy/角速度），复用 common/KalmanFilter；融合轮速与转换到 base 坐标系后的陀螺仪。当前不积分加速度、不在线估计零偏。`gyro_bias` 需静止测量后填写；噪声配置仅为起点，有有效传感器协方差时优先使用消息值。Cartographer 的此配置关闭原始 IMU 输入，使用融合里程计预测，适合地面近似水平的轮式机器人；背包、坡道或明显俯仰/横滚场景继续使用原配置，不能直接套用此平面模型。不要把 `/tracked_pose` 或 `/trajectory` 反馈进 EKF。

4. 另开容器终端，一行录制标定和评估所需频道（频道名按实际配置修改）：

```bash
mkdir -p ~/.autonomy/artifacts/records && autolink recorder record -i 0 -m 0 -o "$HOME/.autonomy/artifacts/records/wheel_imu_$(date +%Y%m%d-%H%M%S).record" -c /scan /wheel/odometry /imu /odometry/filtered /tf /tf_static
```

先静止 60 秒，再采集直行、转弯及回到起点的一段路线，最后 Ctrl-C 正常结束录制。保留原始轮速和 IMU，才能复查零偏、时间同步及融合效果。静止段的陀螺仪必须先转换到 base 坐标系，再统计 Z 轴均值作为 `gyro_bias`；该静止段的方差不能直接代替行驶中的轮速噪声。将静止段、运动段的起止时间与录包一起保存。

执行 `autolink recorder info /absolute/robot.record`，确认 `is_complete: true`，且上面六个频道均有消息。若未录到 `/tf_static`，须提供真实标定 YAML，扫描时添加 `--static-transforms /absolute/robot_static_transform.yaml`。独立真值应单独保存，不能使用 Cartographer 自己的输出充当真值。

5. 使用刚录好的文件，一行执行前端参数扫描：

```bash
python3 autonomy/localization/tools/tune_cartographer.py --record /absolute/robot.record --seconds 60
```

默认在 `voxel_filter_size = 0.025/0.02/0.03`、`translation_weight = 10/5/20` 上比较 9 组，保留当前旋转权重，关闭周期全局优化和最终优化以观察前端。每组直接离线读取 record，不创建 Autolink 节点，不影响正在运行的建图。博物馆数据验证时添加 `--config autonomy/localization/conf/cartographer/backpack_2d.lua`；这只能验证调参流程，不能验证轮速融合精度。

结果保存在 `~/.autonomy/artifacts/cartographer-tuning/<时间-进程号>/`：`results.csv/json`、配置快照、`local.tum`、`trajectory.tum`、`state.pbstream`；日志在 `~/.autonomy/log/cartographer-tuning/`。`realtime_factor > 1` 表示整个 SLAM 计算（含后台收尾、最终优化）快于录包时长；`processing_realtime_factor` 只统计输入处理阶段，不能代表后台已经跟上。前端步进 P95、后台任务收尾等待和最终优化耗时分别统计。这些耗时不包含在线消息乱序缓冲、可视化、地图发布和结果文件导出，不能当成在线端到端延迟。

6. 选好前端参数后写入一份自己的 Lua 配置，再比较后端：

```bash
python3 autonomy/localization/tools/tune_cartographer.py --record /absolute/robot.record --config /absolute/my_robot.lua --phase global --seconds 0
```

默认比较 `min_score = 0.55/0.60` 和 `sampling_ratio = 0.3/0.1`，基础配置必须开启周期全局优化。使用 `--ground-truth /absolute/ground_truth.tum` 可计算 SE(2) 对齐后的 XY ATE 和 1 秒 XY/偏航 RPE；需要 Python NumPy。真值须与输出的 `tracking_frame` 对应同一刚体原点、同一时钟；关联容差为 50 ms。无独立真值时只报告性能，不给出厘米级精度结论，也不自动覆盖生产配置。用完整数据及另一段路线复验后再采用候选值。
