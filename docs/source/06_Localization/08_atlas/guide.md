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
| `/tf` | 默认发布 `map→odom` 动态变换 |

当前实现仍不发布：

- LiDAR registered cloud 或 cloud map
- occupancy grid
- loop marker

Atlas 默认订阅 `/tf` 和 `/tf_static`，在图像时刻查询
`odom→base_link→camera_link`（也兼容直接 `odom→camera_link`），再由
视觉位姿计算并发布 `map→odom`。输入变换缺失或与图像时间相差超过 100 ms
时会跳过本次 TF 发布；可用 `--atlas_publish_tf=false` 关闭该输出。
依赖占用栅格的导航模块仍需显式适配或选择具备相应输出的后端。

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
