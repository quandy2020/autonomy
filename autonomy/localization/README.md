# localization

`autonomy.localization` 是定位进程入口，由 `LocalizationServer` 选择后端。

## 当前可用状态

| 后端 | 当前状态 | 当前提供的启动方式 |
|------|----------|--------------------|
| Cartographer | 可用 | `localization.launch`、`localization_cartographer.launch`，均为 AutoSim 2D |
| Atlas | 可用；视觉 SLAM | `localization_atlas.launch`，默认单目 |
| Argus | 不可用 | 源码目录已不存在，`localization_argus.launch` 当前会启动失败 |

Atlas 已完全定位为以相机为核心的视觉 SLAM 方案。单目、双目和 RGB-D 模式只消费图像；惯性模式是在视觉 SLAM 上增加 IMU 约束形成 VIO。Atlas 不包含 LiDAR、LIO 或 LIVO 数据链路。

当前源码树的实际结构：

```text
autonomy/localization/
├── CMakeLists.txt
├── localization_main.cpp
├── localization_server.cpp
├── localization_server.hpp
├── argus_node.cpp              # 保留的适配代码；当前不参与构建
├── argus_node.hpp
├── atlas/                      # 相机视觉 SLAM / 可选 VIO
├── cartographer/
├── conf/
│   ├── atlas/
│   └── cartographer/
└── launch/
    ├── localization.launch
    ├── localization_cartographer.launch
    ├── localization_atlas.launch
    └── localization_argus.launch   # 失效占位，当前不可运行
```

## 构建

在宿主机执行：

```bash
docker exec -w /workspace/autonomy SpaceHero \
  colcon build --packages-up-to autonomy
```

进入容器后的公共环境：

```bash
docker exec -it SpaceHero /bin/bash
cd /workspace/autonomy/src/autonomy

export PATH=/workspace/autonomy/build/autonomy/bin:/usr/local/bin:/usr/bin:/bin
export LD_LIBRARY_PATH="/workspace/autonomy/install/autonomy/lib:/workspace/autonomy/build/autonomy/lib:${LD_LIBRARY_PATH:-}"
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
```

## Atlas 启动

### 默认 launch

`localization_atlas.launch` 的实际默认值：

| 参数 | 默认值 |
|------|--------|
| `--localization_mode` | `atlas` |
| `--atlas_sensor` | `monocular` |
| `--atlas_config` | `autonomy/localization/atlas/config/euroc_mono.yaml` |
| `--atlas_vocabulary` | `autonomy/localization/atlas/config/orb_vocab.fbow` |
| 主图像 | `/camera/rgb/image_raw` |
| 右图像 | `/camera/right/image_raw` |
| 深度图像 | `/camera/depth/image_raw` |
| IMU | `/imu` |

launch 只启动 Atlas 定位进程，不会自动启动相机、数据集播放器或 Autoviz。默认 YAML 是 EuRoC 标定；只有输入图像与该标定、分辨率和帧率匹配时，跟踪结果才有意义。

### 终端 A：启动 Atlas

```bash
docker exec -it SpaceHero /bin/bash
cd /workspace/autonomy/src/autonomy

export PATH=/workspace/autonomy/build/autonomy/bin:/usr/local/bin:/usr/bin:/bin
export LD_LIBRARY_PATH="/workspace/autonomy/install/autonomy/lib:/workspace/autonomy/build/autonomy/lib:${LD_LIBRARY_PATH:-}"
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch

autolink launch start localization_atlas.launch
```

`autolink launch start` 在前台运行。正常启动时日志应包含：

```text
AtlasNode started sensor=monocular
LocalizationServer: atlas backend started
```

配置中缺少标记为 optional 的键时可能打印提示，例如 `Camera1.k3 optional parameter does not exist`；只要 AtlasNode 成功启动，这类提示不是启动失败。

### 终端 B：检查与停止

```bash
docker exec -it SpaceHero /bin/bash
cd /workspace/autonomy/src/autonomy

export PATH=/workspace/autonomy/build/autonomy/bin:/usr/local/bin:/usr/bin:/bin
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch

autolink launch list
autolink --wait 3 channel list -v
```

停止：

```bash
autolink launch stop localization_atlas.launch
```

停止后确认没有残留进程：

```bash
pgrep -af "autonomy.localization|autolink launch"
```

无输出表示进程已清理。

## Atlas 传感器模式

AtlasNode 当前实现六种模式：

| `--atlas_sensor` | 必需输入 |
|--------------------|----------|
| `monocular` | 主图像 |
| `stereo` | 主图像 + 右图像 |
| `rgbd` | 主图像 + 深度图像 |
| `monocular_inertial` | 主图像 + IMU |
| `stereo_inertial` | 主图像 + 右图像 + IMU |
| `rgbd_inertial` | 主图像 + 深度图像 + IMU |

内置配置：

| 配置文件 | 适用模式 | 范围 |
|----------|----------|------|
| `euroc_mono.yaml` | `monocular` | EuRoC 单目 |
| `euroc_mono_inertial.yaml` | `monocular_inertial` | EuRoC 单目惯性 |
| `euroc_stereo_inertial.yaml` | `stereo`、`stereo_inertial` | EuRoC 双目或双目惯性 |
| `tum_rgbd.yaml` | `rgbd` | TUM RGB-D Freiburg1；不能直接用于 Freiburg2/3 |
| `tum_vi_stereo_inertial.yaml` | `stereo_inertial` | TUM-VI 512 双目惯性 |
| `kitti_00_02.yaml` | `stereo` | KITTI odometry 00–02 |
| `tartanair_rgbd_inertial.yaml` | `rgbd_inertial` | TartanAir V2 合成接口验证；当前数据格式尚不兼容 |

切换模式时必须同时修改：

1. `--atlas_sensor`
2. `--atlas_config`
3. 对应图像和 IMU 话题
4. 相机内参、畸变、分辨率、帧率及相机—IMU 外参

### 不修改 launch，直接指定模式

例如使用 KITTI 标定的双目模式：

```bash
autonomy.localization \
  --localization_mode=atlas \
  --atlas_sensor=stereo \
  --atlas_config=autonomy/localization/atlas/config/kitti_00_02.yaml \
  --atlas_vocabulary=autonomy/localization/atlas/config/orb_vocab.fbow \
  --atlas_image_topic=/camera/left/image_raw \
  --atlas_right_image_topic=/camera/right/image_raw
```

RGB-D：

```bash
autonomy.localization \
  --localization_mode=atlas \
  --atlas_sensor=rgbd \
  --atlas_config=autonomy/localization/atlas/config/tum_rgbd.yaml \
  --atlas_vocabulary=autonomy/localization/atlas/config/orb_vocab.fbow \
  --atlas_image_topic=/camera/rgb/image_raw \
  --atlas_depth_image_topic=/camera/depth/image_raw
```

双目惯性：

```bash
autonomy.localization \
  --localization_mode=atlas \
  --atlas_sensor=stereo_inertial \
  --atlas_config=autonomy/localization/atlas/config/euroc_stereo_inertial.yaml \
  --atlas_vocabulary=autonomy/localization/atlas/config/orb_vocab.fbow \
  --atlas_image_topic=/camera/left/image_raw \
  --atlas_right_image_topic=/camera/right/image_raw \
  --atlas_imu_topic=/imu
```

直接启动的进程使用 `Ctrl-C` 停止。

## Atlas 输入要求

| 输入 | AutoMsgs 类型 | 使用条件 |
|------|---------------|----------|
| 主图像 | `sensor_msgs/Image` | 所有模式 |
| 右图像 | `sensor_msgs/Image` | stereo / stereo_inertial |
| 深度图像 | `sensor_msgs/Image` | rgbd / rgbd_inertial |
| IMU | `sensor_msgs/Imu` | 三种 inertial 模式 |

注意：

- 每条消息必须带有效 `header.stamp`。
- 双目左右图时间差必须不大于约 10 ms。
- RGB-D 彩色图与深度图时间差必须不大于约 30 ms。
- 支持的图像编码包括 `mono8`、`mono16`、`32FC1`、`bgr8`、`rgb8`、`bgra8` 和 `rgba8`。
- 图像尺寸、内参、畸变模型及深度尺度必须与 YAML 一致。
- 惯性模式要求 IMU 时间戳单调，并且噪声和外参配置正确。

## Atlas 输出

Atlas 只发布以下三个话题：

| 话题 | AutoMsgs 类型 | frame | 发布时机 |
|------|---------------|-------|----------|
| `/atlas/odometry` | `nav_msgs/Odometry` | `map` → `camera` | 每个有效跟踪帧 |
| `/atlas/trajectory` | `nav_msgs/Path` | `map` | 约每秒更新 |
| `/atlas/map_points` | `sensor_msgs/PointCloud2` | `map` | 约每秒更新且地图点非空时 |

当前 Atlas：

- 不消费 LiDAR。
- 不发布 TF。
- 不发布 occupancy grid。
- 不发布 registered cloud、稠密 cloud map 或 loop markers。

下游如果依赖 `map→odom→base_link`，需要单独实现明确的 TF 适配层，不能假定 Atlas 已经发布 TF。

## 使用 Autoviz 查看 Atlas

当前仓库没有 `atlas.autoviz` 预设，`argus.autoviz` 也不适用于 Atlas。启动空白 Autoviz：

```bash
docker exec -it SpaceHero /bin/bash
cd /workspace/autonomy

./build/autonomy/bin/autoviz
```

在界面中设置：

1. Global → Fixed Frame：`map`
2. 添加 Odometry：`/atlas/odometry`
3. 添加 Path：`/atlas/trajectory`
4. 添加 PointCloud2：`/atlas/map_points`
5. Image 面板选择实际主图像话题，例如 `/camera/rgb/image_raw`

Atlas 的三个输出本身都使用 `map` frame，因此显示它们不依赖 Atlas 发布 TF。RobotModel 或其他传感器 frame 是否能显示，取决于系统中是否有其他节点提供相应 TF。

若 Autoviz 没有数据，先检查：

```bash
autolink --wait 3 channel list -v
```

只有输入图像存在且 Atlas 获得有效跟踪后，才会出现有效里程计、轨迹和地图点。

## 离线数据集回放

### 回放边界

`localization_atlas.launch` 消费 Autolink 实时话题。当前 `autonomy.tools.datasets` 只支持 Oxford Spires 的 LiDAR/IMU 格式，不是 Atlas 的 EuRoC/TUM 图像发布器，因此不能用它直接驱动 Atlas launch。

下述 runner 直接从磁盘读取图像和 IMU，并调用 Atlas 核心库：

- 不创建 Autolink node，也不发布 `/atlas/odometry`、`/atlas/trajectory` 或 `/atlas/map_points`。
- 不能直接用 Autoviz 做实时话题显示；若要走 `localization_atlas.launch` 和 Autoviz，仍需另行实现发布对应 AutoMsgs 话题的数据集播放器。
- 不提供 `--rate` 参数。双目、惯性、TUM RGB-D 和 KITTI runner 会按数据时间间隔节流；`atlas_mono_euroc` 不节流，会尽快处理完整序列。
- 轨迹文件是 runner 停止时保存的离线结果，不等同于 `/atlas/trajectory` 话题。

### 构建离线 runner

Atlas 自带离线示例程序，但默认配置项 `ATLAS_BUILD_EXAMPLES=OFF`，普通干净构建不会生成这些可执行文件。启用示例：

```bash
docker exec -w /workspace/autonomy SpaceHero \
  colcon build --packages-up-to autonomy \
  --cmake-args -DATLAS_BUILD_EXAMPLES=ON
```

启用后，可执行文件位于：

```text
/workspace/autonomy/build/autonomy/bin/
├── atlas_mono_euroc
├── atlas_stereo_euroc
├── atlas_mono_inertial_euroc
├── atlas_stereo_inertial_euroc
├── atlas_stereo_inertial_tum_vi
├── atlas_rgbd_tum
├── atlas_stereo_kitti
└── atlas_rgbd_inertial_tartanair
```

### 当前数据就绪状态

| 数据集 | Atlas 模式 | 容器内数据 | 当前状态 |
|--------|------------|------------|----------|
| EuRoC | 单目、双目、单目惯性、双目惯性 | `/data/slam/EuRoC/data` 下 11 个完整序列 | 可回放；MH_01 前 80 帧四种模式均已烟测 |
| TUM RGB-D | RGB-D | `/data/slam/TUM` 下 Freiburg1 压缩包 | 可解压回放；`freiburg1_desk` 完整序列 573/573 帧有效 |
| KITTI odometry | 双目 | `/data/slam/kitti/odometry/sequences/00`–`02` | 可回放；序列 00 前 20 帧烟测为 20/20 |
| TUM-VI | 双目惯性 | `/data/slam/TUM-VI` | 不可回放；只有不完整分片，没有可解压序列 |
| TartanAir V2 | RGB-D 惯性 | `/data/slam/tartanair_v2` | 不可回放；当前数据缺少深度目录，下载包是 PNG 深度，而 reader 只接受 float32 NPY |

所有命令都在容器中执行：

```bash
docker exec -it SpaceHero /bin/bash
cd /workspace/autonomy/src/autonomy
```

### EuRoC：四种视觉模式

以下命令以 `MH_01_easy` 为例。先从 `cam0/data.csv` 生成每行一个纳秒时间戳的列表；不能把 CSV 直接传给 runner：

```bash
awk -F, '!/^#/ {print $1}' \
  /data/slam/EuRoC/data/MH_01_easy/mav0/cam0/data.csv \
  > /tmp/MH_01_easy.timestamps
```

单目：

```bash
/workspace/autonomy/build/autonomy/bin/atlas_mono_euroc \
  autonomy/localization/atlas/config/orb_vocab.fbow \
  autonomy/localization/atlas/config/euroc_mono.yaml \
  /data/slam/EuRoC/data/MH_01_easy \
  /tmp/MH_01_easy.timestamps
```

`atlas_mono_euroc` 是唯一不接受轨迹输出参数的 runner，并且不会按数据时间节流。

双目：

```bash
/workspace/autonomy/build/autonomy/bin/atlas_stereo_euroc \
  autonomy/localization/atlas/config/orb_vocab.fbow \
  autonomy/localization/atlas/config/euroc_stereo_inertial.yaml \
  /data/slam/EuRoC/data/MH_01_easy \
  /tmp/MH_01_easy.timestamps \
  /tmp/atlas_mh01_stereo.txt
```

单目惯性：

```bash
/workspace/autonomy/build/autonomy/bin/atlas_mono_inertial_euroc \
  autonomy/localization/atlas/config/orb_vocab.fbow \
  autonomy/localization/atlas/config/euroc_mono_inertial.yaml \
  /data/slam/EuRoC/data/MH_01_easy \
  /tmp/MH_01_easy.timestamps \
  /data/slam/EuRoC/data/MH_01_easy/mav0/imu0/data.csv \
  /tmp/atlas_mh01_mono_inertial.txt
```

双目惯性：

```bash
/workspace/autonomy/build/autonomy/bin/atlas_stereo_inertial_euroc \
  autonomy/localization/atlas/config/orb_vocab.fbow \
  autonomy/localization/atlas/config/euroc_stereo_inertial.yaml \
  /data/slam/EuRoC/data/MH_01_easy \
  /tmp/MH_01_easy.timestamps \
  /data/slam/EuRoC/data/MH_01_easy/mav0/imu0/data.csv \
  /tmp/atlas_mh01_stereo_inertial.txt
```

将命令中的 `MH_01_easy` 同时替换为其他已解压序列名即可切换数据。双目和惯性 runner 的可选输出为 EuRoC/TUM 格式：`timestamp tx ty tz qx qy qz qw`。

### TUM RGB-D：Freiburg1

`tum_rgbd.yaml` 使用 Freiburg1 标定，不能直接用于 Freiburg2/3。以 `freiburg1_desk` 为例，先解压：

```bash
tar -xzf /data/slam/TUM/rgbd_dataset_freiburg1_desk.tgz \
  -C /data/slam/TUM
```

TUM 的 RGB 和深度时间戳不同。使用 Atlas 自带脚本生成一对一最近时间戳 association：

```bash
python3 autonomy/localization/atlas/examples/associate_tum_rgbd.py \
  /data/slam/TUM/rgbd_dataset_freiburg1_desk/rgb.txt \
  /data/slam/TUM/rgbd_dataset_freiburg1_desk/depth.txt \
  --output /tmp/fr1_desk.associations.txt
```

运行 RGB-D 回放并保存 TUM 格式轨迹：

```bash
/workspace/autonomy/build/autonomy/bin/atlas_rgbd_tum \
  autonomy/localization/atlas/config/orb_vocab.fbow \
  autonomy/localization/atlas/config/tum_rgbd.yaml \
  /data/slam/TUM/rgbd_dataset_freiburg1_desk \
  /tmp/fr1_desk.associations.txt \
  /tmp/atlas_freiburg1_desk.txt
```

association 每行格式为 `rgb_timestamp rgb/path depth_timestamp depth/path`。生成脚本已在 `freiburg1_desk` 上与 573 行基准逐行核对一致。

### KITTI odometry：序列 00–02

以序列 00 为例：

```bash
/workspace/autonomy/build/autonomy/bin/atlas_stereo_kitti \
  autonomy/localization/atlas/config/orb_vocab.fbow \
  autonomy/localization/atlas/config/kitti_00_02.yaml \
  /data/slam/kitti/odometry/sequences/00 \
  /tmp/atlas_kitti_00.txt
```

切换到 01 或 02 时，同时替换序列目录和输出文件名。reader 使用 `times.txt`、`image_0/` 和 `image_1/`；不会读取序列内的 `calib.txt`，相机标定完全来自 `kitti_00_02.yaml`。当前 YAML 不能用于其他 KITTI 序列。

如需评测序列 00：

```bash
python3 autonomy/localization/atlas/examples/evaluate_kitti.py \
  /data/slam/kitti/odometry/poses/00.txt \
  /tmp/atlas_kitti_00.txt \
  /tmp/atlas_kitti_00_eval
```

轨迹是 KITTI 格式，每行一个 `3x4` 位姿矩阵。评测目录包含统计 JSON、对齐轨迹 CSV 和 PNG。

### 当前不能播放的数据

TUM-VI 当前只有 `/data/slam/TUM-VI/dataset-room1_512_16.tar.part00`–`part15` 及一个未完成文件；分片大小不一致，也没有解压后的 `mav0/`。不要拼接或运行 `atlas_stereo_inertial_tum_vi`，应先重新取得完整归档并校验后再解压。

TartanAir V2 当前的 RGB 和 IMU 数据不构成可用的 RGB-D 惯性序列：现有目录没有 `depth_lcam_front/`，而正在取得的官方深度包包含 PNG；`atlas_rgbd_inertial_tartanair` 当前只接受 `*_lcam_front_depth.npy` 的 little-endian、C-order float32 米制深度。必须先明确 PNG 编码与尺度并适配 reader，再做正向烟测；当前 runner 的存在不代表该数据集已经可用。

更完整的轨迹评测方法见 [atlas/docs/06-testing-and-benchmarking.md](atlas/docs/06-testing-and-benchmarking.md)。

## Atlas 故障排查

| 现象 | 检查项 |
|------|--------|
| launch 找不到 | `AUTOLINK_LAUNCH_PATH` 是否指向 `autonomy/localization/launch` |
| 配置或词表打不开 | 路径是否相对 `/workspace/autonomy/src/autonomy`，文件是否存在 |
| 进程正常但没有输出 | 输入图像话题是否存在；图像格式、尺寸和标定是否匹配 |
| 双目/RGB-D 没有输出 | 两路时间戳是否满足 10 ms / 30 ms 同步容差 |
| VIO 不稳定 | IMU 时间、单位、频率、噪声及相机—IMU 外参 |
| 只有 odometry，没有轨迹或地图点 | 轨迹/地图约每秒发布；地图初始化前可能没有地图点 |
| Autoviz 看不到输出 | Fixed Frame 是否为 `map`，Display 是否选择正确 `/atlas/*` 话题 |
| 导航栈没有 TF | 符合当前设计；Atlas 不发布 TF，需要单独适配 |

## Cartographer

当前提供的 launch 只覆盖 AutoSim 2D，不是现成的 3D profile。

启动：

```bash
docker exec -it SpaceHero /bin/bash
cd /workspace/autonomy/src/autonomy

mkdir -p data/maps
export PATH=/workspace/autonomy/build/autonomy/bin:/usr/local/bin:/usr/bin:/bin
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch

autolink launch start localization_cartographer.launch
```

默认输入：

- `/scan`
- `/odom`
- `/tf`
- `/tf_static`

默认 `autosim_2d.lua` 不消费 IMU。主要输出为 `/map`、`/submap_list`、`/tracked_pose` 和 `/tf`。launch 停止时会保存 `data/maps/cartographer.pbstream`。

## Argus

当前源码树没有 `autonomy/localization/argus/` 和 `conf/argus/`，CMake 会以 `AUTONOMY_HAS_ARGUS=0` 构建。`localization_argus.launch` 是未清理的旧占位文件，当前执行只会报：

```text
Argus backend is not available in this build.
```

因此当前 README 不提供 Argus 启动、数据集或 Autoviz 流程。旧 `install/` 中残留的 Argus YAML 或可执行文件也不能视为当前源码能力。

## CLI 参数

| 后端 | 参数 |
|------|------|
| 通用 | `--localization_mode` |
| Cartographer | `--configuration_directory`、`--configuration_basename`、`--load_state_filename`、`--save_state_filename` |
| Atlas | `--atlas_config`、`--atlas_vocabulary`、`--atlas_sensor`、`--atlas_image_topic`、`--atlas_right_image_topic`、`--atlas_depth_image_topic`、`--atlas_imu_topic` |
| Argus | 参数仍保留在 CLI，但后端当前未编入，不能使用 |

## Atlas 进一步文档

- [Atlas 总览](atlas/README.md)
- [架构](atlas/docs/02-architecture.md)
- [构建与 CMake](atlas/docs/03-build-and-cmake.md)
- [配置](atlas/docs/04-configuration.md)
- [AutoMsgs 集成](atlas/docs/05-automsgs-integration.md)
- [测试与评测](atlas/docs/06-testing-and-benchmarking.md)
