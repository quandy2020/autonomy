(atlas-guide)=
# Atlas 视觉 / VIO SLAM

## 背景

Atlas 是 autonomy 工程内的 ORB-SLAM3 移植与模块化实现。运行入口是 `autonomy.localization --localization_mode=atlas`，支持单目、双目、RGB-D 以及三种对应的惯性模式。

Atlas 只处理相机和可选 IMU；LiDAR LIO/LIVO 由 Argus 或 Cartographer 承担。

## 支持的模式

| `atlas_sensor` | 必需输入 |
|------------------|----------|
| `monocular` | 主图像 |
| `stereo` | 主图像 + 右图像 |
| `rgbd` | 主图像 + 深度图像 |
| `monocular_inertial` | 主图像 + IMU |
| `stereo_inertial` | 主图像 + 右图像 + IMU |
| `rgbd_inertial` | 主图像 + 深度图像 + IMU |

鱼眼是 `monocular`、`stereo` 及其惯性模式使用的相机模型，不是独立的
`atlas_sensor` 模式。TUM-VI room1 是鱼眼双目 + IMU 的真实数据集验证序列。

| 能力 | 核心/API | 离线示例与配置 | 数据集验证 |
|------|----------|----------------|------------|
| 纯视觉 | 已支持 | EuRoC 单目、KITTI 双目、TUM RGB-D | 已验证 |
| 单目 + IMU | 已支持 | EuRoC | EuRoC 已验证 |
| 双目 + IMU | 已支持 | EuRoC、TUM-VI | EuRoC、TUM-VI room1 已验证 |
| RGB-D + IMU | 已支持 | TartanAir | P000、P003 已完成合成数据集验证 |

上表将接口、离线入口和数据集验证分开统计，存在示例不代表精度已经验证。

## 构建与启动

Atlas 随 `autonomy` 包构建：

```bash
docker exec -w /workspace/autonomy SpaceHero \
  colcon build --packages-up-to autonomy
```

使用 Autolink launch：

```bash
cd /workspace/autonomy/src/autonomy
export PATH=/workspace/autonomy/build/autonomy/bin:/usr/local/bin:/usr/bin:/bin
export AUTOLINK_LAUNCH_PATH=/workspace/autonomy/src/autonomy/autonomy/localization/launch

autolink launch start localization_atlas.launch
autolink launch list
autolink launch stop localization_atlas.launch
```

默认 profile 是 `monocular` + `atlas/config/euroc_mono.yaml` + `atlas/config/orb_vocab.fbow`，主图像话题为 `/camera/rgb/image_raw`。

## 输入参数

| flag | 说明 |
|------|------|
| `--atlas_config` | 相机、IMU 与算法设置 YAML |
| `--atlas_vocabulary` | FBOW 词表 |
| `--atlas_sensor` | 上表中的传感器模式 |
| `--atlas_image_topic` | 主图像 |
| `--atlas_right_image_topic` | 双目右图像 |
| `--atlas_depth_image_topic` | RGB-D 深度图像 |
| `--atlas_imu_topic` | 惯性模式的 IMU |

launch 中配置了所有输入话题，AtlasNode 会按 `atlas_sensor` 只创建所需订阅。

## 内置配置

| 配置 | 对应模式 |
|------|----------|
| `euroc_mono.yaml` | `monocular` |
| `euroc_mono_inertial.yaml` | `monocular_inertial` |
| `euroc_stereo_inertial.yaml` | `stereo_inertial` |
| `tum_vi_stereo_inertial.yaml` | `stereo_inertial`（TUM-VI room1 已验证） |
| `tum_rgbd.yaml` | `rgbd` |
| `tartanair_rgbd_inertial.yaml` | `rgbd_inertial`（P000、P003 已验证） |
| `kitti_00_02.yaml` | `stereo` |

这些 YAML 是数据集标定，不是通用真实相机配置。接入实车或 AutoSim 时，必须提供匹配的分辨率、内参、畸变、帧率以及相机—IMU 外参，并同时修改模式和话题。

## TartanAir RGB-D + IMU 验证

评测使用最近邻时间关联和固定尺度 SE(3) 刚体对齐，不使用 Sim(3) 修正
RGB-D 尺度。CPU/RSS 由 `pidstat` 每秒采样。

| 序列 | samples/accepted/valid | 匹配（估计/GT 覆盖率） | ATE RMSE | CPU 平均/峰值 | Max RSS |
|------|------------------------|-------------------------|----------|-----------------|---------|
| P003 | 152/152/150 | 150（100%/98.6842%） | 0.0212131 m | 58.839%/82% | 244160 KiB |
| P000 | 116/116/108 | 70（100%/60.3448%） | 0.0202723 m | 39.308%/74% | 180164 KiB |

P003 是覆盖率更完整的主结果；P000 因前段重建只导出 70 个有效轨迹位姿，
因此其低 ATE 不能脱离 60.3448% 的 GT 覆盖率单独比较。结果产物位于
`build/atlas_benchmark_results/vio_modes/rgbd_inertial_tartanair_postfix/`。
TartanAir 是合成数据集，以上结果验证离线接口、尺度和数值稳定性，不代表
真实传感器完成了标定或精度验收。

## 输出

| 话题 | 内容 |
|------|------|
| `/atlas/odometry` | 当前相机位姿里程计 |
| `/atlas/trajectory` | 已跟踪轨迹 `Path` |
| `/atlas/map_points` | 当前稀疏地图点 `PointCloud2` |
| `/atlas/dense_map_points` | 启用稠密建图后的彩色地图 `PointCloud2` |
| `/atlas/occupancy_grid` | 启用稠密建图后的导航投影 `OccupancyGrid` |
| `/atlas/optimized_trajectory` | 优化后的关键帧 `Path` |
| `/atlas/loop_closures` | 回环约束 `MarkerArray` |
| `/tf` | 默认发布 `map→odom` 动态变换 |

Atlas 不产生 LiDAR registered cloud 或 LiDAR cloud map。
占用栅格和稠密点云由实际 RGB-D/双目建图流水线发布，需要
`--atlas_enable_dense_mapping=true` 和非空输出话题；发布成功不等于
已经融合探索消费者接收的全部输入，也不构成保存地图的回执。

Atlas 默认订阅 `/tf` 和 `/tf_static`，在图像时刻查询
`odom→base_link→camera_link`（也兼容直接 `odom→camera_link`），再由
视觉位姿计算并发布 `map→odom`。输入变换缺失或与图像时间相差超过 100 ms
时会跳过本次 TF 发布；可用 `--atlas_publish_tf=false` 关闭该输出。
非惯性 RGB-D 的世界坐标从第一帧相机初始化，栅格可能采用 XZ 投影。
导航消费者须检查消息里的栅格原点姿态及世界坐标关系，不能仅把
`frame_id` 改成 `map` 就当作已经完成对齐。

## 隔离实例与 RGB-D 验证

2026-11-18 补齐 `LocalizationOptions` → `AtlasBackend` → `AtlasNode::Options`
和原生 CLI 的输出话题转发。新增参数为
`--atlas_odometry_topic`、`--atlas_trajectory_topic`、
`--atlas_optimized_trajectory_topic`、`--atlas_map_points_topic`、
`--atlas_tracking_image_topic`、`--atlas_loop_closures_topic` 和
`--atlas_ground_truth_trajectory_topic`。原来的稠密地图、特征匹配、占用
栅格与 TF 参数继续使用；所有默认话题与已有 AtlasNode 一致。
这些参数复用已有 AutoMsgs 类型，没有新增传输消息。

验证实例把全部输入/输出放到同一 `/validation/exploration/…` 前缀，并为
AutoSim 和 Atlas 分别设置 `AUTOSIM_INSTANCE_LOCK_PATH` 与
`AUTONOMY_ATLAS_INSTANCE_LOCK_PATH`。只停止本次创建的子进程；已有
AutoSim 实例继续运行。纯 RGB-D 模式不启用 IMU 和 LiDAR。

`scripts/validate_rgbd_localization.py` 等待实际 `CameraInfo`、注册 RGB8、
米单位 `32FC1` 深度及静态 TF，再生成相机 YAML。内参来自消息，安装高度
来自 `base_footprint→相机` 的实际静态变换链。浮点参数保留 `.0` 或小数，
避免 OpenCV 将整数形式的主点坐标判为非 real。图像尺寸和 `Camera.fps`
按 Atlas 的解析约定输出为整数。即使关闭 GUI，Atlas 仍解析十项必需的
`Viewer.*` 浮点参数，因此验证配置同时提供这些设置。没有沿用 TUM 内参、
畸变或 5000 倍深度缩放。

验证时显式关闭 Atlas 的 GT 输入和 GT 轨迹输出，同时关闭 TF 发布；
独立记录器保留 GT 栅格和位姿用于之后的评价。Atlas 的估计位姿和占用
栅格由它自己的实际 RGB-D 流水线产生。脚本记录原始消息、程序日志、
标定与词表摘要、退出状态以及非未知栅格数量。

在当前 SpaceHero 中，定位依赖配置与四个实际应用的构建及 `--help`
已通过。原始证据为 `docs/validation/2026-11-18-v2.52-localization-applications.json`
和补齐话题后的 `2026-11-18-v2.53-localization-applications.json`。
这是应用构建证据；实际 RGB-D 运行有独立记录，完整探索任务及真值覆盖
比较也必须分别验证，不能从 CLI 成功推导。

v2.57 已通过真实 Habitat + Atlas RGB-D 静止运行：640×480、10 Hz，
2500 个模拟步，收到 RGB、深度和 CameraInfo 各 500 条；Atlas 估计
里程计 498 条、轨迹 249 条、算法占用栅格 50 条，最终地图包含 3073 个
已知格子。Atlas 正常退出，安装高度从静态 TF 得到 0.21 m。
四项标定配置回归通过，并将 23 个原始消息、配置和日志文件按摘要导出。
证据为 `docs/validation/2026-11-18-v2.57-rgbd-localization-attempt.json`，
原始记录位于同名前缀的 `-recording/recording/` 子目录。
运行没有发送运动指令；起止真值位姿相同，不能用它验证移动跟踪精度、
探索覆盖、参考平滑性、避障或闭环保存。

## 原生定位状态与探索关联

`--atlas_localization_status_topic` 可配置原生 automsgs `DiagnosticArray`
输出；空字符串关闭该输出，不改变原有话题配置。每次完成或失败的相机
跟踪尝试都发布 `atlas/localization` 状态，消息 Header 保留该次尝试的
采集时间及 Atlas 世界帧。

只有实际跟踪结果有效、地图身份有效、度量坐标已就绪，并且结果时间与
该次采集时间在 native nanosecond 表示下完全相同时，状态才为 `OK`。
MessageAdapter 的定位和路径 Header 保留实际输入的整数秒/纳秒；内部
ORB 跟踪仍使用 double 时间。不能通过 double 重建 epoch nanoseconds
后关联深度，因为大 epoch 时间的浮点精度可能产生数百纳秒偏差。
失败尝试不会把上一帧仍有效的结果变成当前帧健康状态。状态与 Odometry
可以先后到达，消费者必须按时间和帧名关联；不能把最新状态配给旧深度。

`values` 保留版本标识 `autonomy.atlas.tracking.v1`、实际 sensor mode、
camera frame、odometry topic、跟踪状态、有效性和实际 Atlas map id、
coordinate epoch、destructive epoch。`hardware_id` 区分进程/启动实例，
避免重启后的相同 map id 被当作原有坐标世代。这里的地图身份是后端原始
值，消费者仍需建立非零 map epoch 并处理重置及坐标变更。

这些状态由真实 MapBuilder 产生，补充普通 Odometry 缺少的健康及坐标身份。
它们不包含底盘外参标定，也不证明地图融合、覆盖或保存。探索平台仍需
使用实际静态安装链组合相机与底盘位姿，并对实际测量和地图处理结果建立
关联。RGB-D 验证脚本记录原始状态消息，检查至少五组原始深度、健康状态
和估计位姿的相同整数时间戳；后两者的地图/相机帧也必须一致。该项验证
仍属于定位建图接入，不代表完整探索通过。

v2.63 的真实 Habitat + Atlas RGB-D 运行已经验证上述整数时间关联：
500 条相机输入中有 498 组健康状态、估计位姿与原始深度精确对应，
算法地图含 3073 个已知格子，Atlas 正常退出。该验证仍是静止定位建图。
原始记录和 26 文件摘要校验位于
`docs/validation/2026-11-18-v2.63-rgbd-acquisition-recording/` 及同版本
`-export.json`，任务整体接入状态见 Tasks 下的 native exploration 验证文档。

## 常见问题

| 现象 | 先查 |
|------|------|
| 启动即失败 | 配置文件和 FBOW 词表路径是否存在 |
| 没有里程计 | `atlas_sensor` 与图像/深度/IMU 话题是否一致 |
| 双目或 RGB-D 无跟踪 | 左右图/深度时间戳、分辨率和标定是否匹配 |
| VIO 不稳定 | IMU 单位、频率、时间同步、噪声参数和外参 |
| 导航栈没有 TF | 检查 `atlas_publish_tf`、TF 话题及 `odom→camera_link` 变换链 |
| CPU 过高 | 降低图像分辨率/帧率，限制 OpenMP 线程并检查回环频率 |

库结构、AutoMsgs 适配和测试说明见源码 `autonomy/localization/atlas/README.md` 及其 `docs/`。
