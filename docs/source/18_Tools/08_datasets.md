# 8. 数据集发布

实现：`autonomy/tools/datasets`，入口：

```bash
/usr/bin/python3 -m autonomy.tools.datasets
```

该工具把磁盘数据按原始时间轴编码为 AutoMsgs 并发布到 Autolink。视觉数据可驱动 Atlas，Oxford Spires 可驱动 Argus。

## 支持范围

| `--dataset` | 默认 `--atlas-sensor auto` | 数据布局 | 当前机器数据 |
|-------------|----------------------------|----------|--------------|
| `spires` | 不适用 | Oxford Spires LiDAR/IMU/相机 | 可用 |
| `euroc` | `monocular` | EuRoC ASL `mav0/` | 11 个序列可用 |
| `tum-rgbd` | `rgbd` | TUM RGB-D `rgb.txt`、`depth.txt` | Freiburg1 压缩包可解压使用 |
| `tum-vi` | `stereo_inertial` | TUM-VI `mav0/` | 当前下载分片不完整，暂不可运行 |
| `kitti` | `stereo` | KITTI odometry sequence | 00–02 可用 |

TartanAir 尚未接入该发布器：当前深度包是 PNG，而已有 Atlas 离线 reader 只接受 float32 NPY，不能宣称可用。

## 话题约定

| 话题 | AutoMsgs 类型 | 用途 |
|------|---------------|------|
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | Atlas 主图 |
| `/camera/right/image_raw` | `sensor_msgs/Image` | Atlas 右图 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Atlas 深度图 |
| `/imu` | `sensor_msgs/Imu` | Atlas/Argus IMU |
| `/points` | `sensor_msgs/PointCloud2` | Spires LiDAR |
| `/gt/odometry` | `nav_msgs/Odometry` | Spires 真值 |
| `/gt/path` | `nav_msgs/Path` | Spires 真值轨迹 |

视觉发布器逐帧交错发布同步图像，并在惯性模式的每个图像时刻前发布截至该时刻的 IMU。Atlas 的同步容差为双目 10 ms、RGB-D 30 ms；不要把左右图分成两个批次发布。

## 宿主机挂载

`run_autonomy.py` 将数据集根挂到容器 `/data`。修改挂载后必须重建容器：

```bash
export AUTONOMY_DATASETS=/path/to/datasets
AUTONOMY_FORCE_RECREATE=1 python3 docker/run_autonomy.py
```

语法为 `HOST` 或 `HOST:CONTAINER[:MODE]`，CLI `--datasets` 优先于环境变量。

## 容器公共环境

```bash
docker exec -it SpaceHero /bin/bash
cd /workspace/autonomy/src/autonomy

export PATH=/workspace/autonomy/build/autonomy/bin:$PATH
export LD_LIBRARY_PATH="/workspace/autonomy/install/autonomy/lib:/workspace/autonomy/build/autonomy/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="/workspace/autonomy/src/autonomy:/workspace/autonomy/build/autonomy/python:/workspace/autonomy/install/autonomy/python:/workspace/autonomy/install/autonomy/lib/python:${PYTHONPATH:-}"
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
```

Isaac 镜像使用 `/usr/bin/python3`，不要依赖交互 shell 的 `python` alias。

## Atlas 三终端工作流

顺序必须是 Atlas → 数据发布器 → Autoviz。Atlas 启动时要加载词表；若先播放，开头图像不会被补发。发布器默认 `--startup-delay 1.0`，但它不能替代确认 Atlas 已出现 `AtlasNode started` 日志。

### 终端 A：启动 Atlas profile

| 数据/模式 | launch |
|-----------|--------|
| EuRoC 单目 | `localization_atlas.launch` |
| EuRoC 双目 | `localization_atlas_euroc_stereo.launch` |
| EuRoC 单目惯性 | `localization_atlas_euroc_mono_inertial.launch` |
| EuRoC 双目惯性 | `localization_atlas_euroc_stereo_inertial.launch` |
| TUM RGB-D Freiburg1 | `localization_atlas_tum_rgbd.launch` |
| TUM-VI 双目惯性 | `localization_atlas_tum_vi_stereo_inertial.launch`；数据完整后使用 |
| KITTI odometry 00–02 | `localization_atlas_kitti_stereo.launch` |

例如：

```bash
autolink launch start localization_atlas_euroc_stereo_inertial.launch
```

### 终端 B：发布数据

发布前可先用 `info` 验证数据目录；该命令只输出 JSON，不发布话题：

```bash
/usr/bin/python3 -m autonomy.tools.datasets info \
  --dataset euroc \
  --path /data/slam/EuRoC/data/MH_01_easy \
  --atlas-sensor stereo_inertial
```

下面每条命令均可直接复制；注释中的 launch 必须先在终端 A 启动。

EuRoC 四种模式：

```bash
# 单目：localization_atlas.launch
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset euroc \
  --path /data/slam/EuRoC/data/MH_01_easy \
  --atlas-sensor monocular \
  --rate 1.0

# 双目：localization_atlas_euroc_stereo.launch
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset euroc \
  --path /data/slam/EuRoC/data/MH_01_easy \
  --atlas-sensor stereo \
  --rate 1.0

# 单目惯性：localization_atlas_euroc_mono_inertial.launch
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset euroc \
  --path /data/slam/EuRoC/data/MH_01_easy \
  --atlas-sensor monocular_inertial \
  --rate 1.0

# 双目惯性：localization_atlas_euroc_stereo_inertial.launch
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset euroc \
  --path /data/slam/EuRoC/data/MH_01_easy \
  --atlas-sensor stereo_inertial \
  --rate 1.0
```

TUM RGB-D：

```bash
# localization_atlas_tum_rgbd.launch
mkdir -p /tmp/atlas_tum_rgbd
tar -xzf /data/slam/TUM/rgbd_dataset_freiburg1_desk.tgz \
  -C /tmp/atlas_tum_rgbd

/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset tum-rgbd \
  --path /tmp/atlas_tum_rgbd/rgbd_dataset_freiburg1_desk \
  --atlas-sensor rgbd \
  --rate 1.0
```

当前 `/data` 按只读方式挂载，不能直接解压到 `/data/slam/TUM`；也可以在宿主机先解压，再重新挂载数据目录。

TUM-VI 双目惯性：

```bash
# localization_atlas_tum_vi_stereo_inertial.launch
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset tum-vi \
  --path /data/slam/TUM-VI/dataset-room1_512_16 \
  --atlas-sensor stereo_inertial \
  --rate 1.0
```

当前机器上的 TUM-VI 下载分片不完整。命令格式有效，但实际运行前必须准备包含完整 `mav0/` 的序列目录。

KITTI 双目：

```bash
# localization_atlas_kitti_stereo.launch
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset kitti \
  --path /data/slam/kitti/odometry/sequences/00 \
  --atlas-sensor stereo \
  --rate 1.0
```

快速冒烟测试可在任一 `publish` 命令末尾追加 `--start 0 --max-frames 100`；完整参数使用 `/usr/bin/python3 -m autonomy.tools.datasets publish --help` 查看。

`--start` 和 `--max-frames` 按主图帧计数；`--rate 1.0` 表示数据集实时速率，`0` 表示尽快发布。Atlas 实时链路建议使用 `1.0`，避免订阅队列被瞬时淹没。视觉数据明确不支持 `--loop`；重新播放前应重启 Atlas，避免时间戳回绕污染状态。

### 终端 C：Autoviz

```bash
cd /workspace/autonomy
./build/autonomy/bin/autoviz \
  --config src/autonomy/autoviz/config/atlas.autoviz
```

该预设以 `map` 为 Fixed Frame，显示：

- `/camera/rgb/image_raw`
- `/atlas/odometry`
- `/atlas/trajectory`
- `/atlas/map_points`
- `/tf` 中由 Atlas 发布的动态 `map → odom`

完整树为 `map → odom → base_link → camera_link`。底盘里程计需在 `/tf` 提供 `odom → base_link`，平台标定需在 `/tf_static` 提供 `base_link → camera_link`；Atlas 用这两项和视觉 `T_map_camera` 计算 `map → odom`。数据集工具本身不伪造底盘 TF，缺少链路时 Atlas 会跳过 TF，但其他 SLAM 输出仍正常。

```bash
autolink channel echo --once /tf
autolink channel echo --once /tf_static
```

停止：先等待发布器结束或按 `Ctrl-C`，再执行：

```bash
autolink launch stop localization_atlas_euroc_stereo_inertial.launch
```

## Oxford Spires / Argus

终端 A：

```bash
autolink launch start localization_argus.launch
```

终端 B：

```bash
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset spires \
  --path /data/Oxford/sequences/2024-03-12-keble-college-02 \
  --rate 1.0
```

Autoviz：

```bash
cd /workspace/autonomy
./build/autonomy/bin/autoviz \
  --config src/autonomy/autoviz/config/argus.autoviz
```

Spires 默认发布图像和真值；使用 `--no-image` 或 `--no-gt` 可关闭。GT 优先 `processed/vilens-slam/gt.tum`，否则使用 `processed/trajectory/gt-tum.txt`。

## 排查

| 现象 | 检查 |
|------|------|
| 发布器启动但 Atlas 无输出 | `--atlas-sensor` 与 launch 是否一致；先确认 Atlas 已启动，再发布 |
| 双目/RGB-D 静默 | 检查右图/深度话题以及时间差；`autolink --wait 3 channel list -v` |
| VIO 不稳定 | IMU CSV、时间单调性及 YAML 相机—IMU 外参 |
| Autoviz 只有图像 | 地图初始化需要若干帧；检查 `/atlas/odometry` 是否已有频率 |
| 播放过快或丢帧 | 使用 `--rate 1.0`，不要对视觉数据使用 `--rate 0` |
| 再次播放后时间异常 | 停止并重启 Atlas；视觉发布器不允许 `--loop` |
| TUM-VI 无法打开 | 当前 `/data/slam/TUM-VI` 是不完整分片，不是 reader 问题 |
| TartanAir 无法选择 | 当前未接入 `--dataset`，深度格式仍待适配 |

`info` 子命令只检查数据布局和帧数，不发布话题。

参见 [06 Localization](../06_Localization/02_quickstart.md) 和 [04 Running · Docker](../04_Running/05_docker_runtime.md)。
