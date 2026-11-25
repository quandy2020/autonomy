# 3. 数据集下载与发布

下载脚本按 `autonomy.tools.datasets` 的目录约定落盘。发布器把磁盘序列按原始时间轴编成 AutoMsgs，推到 Autolink。

脚本：`autonomy/localization/scripts/download_datasets.sh`（转调同目录 `download_datasets.py`）。已存在且非空的文件会跳过，支持断点续传。默认体积为几十到上百 GiB。TartanAir 没有稳定直链，脚本不下载。

## 目录约定

| 组 | 容器路径 |
|----|----------|
| EuRoC | `/data/slam/EuRoC/data/<seq>/mav0/` |
| KITTI | `/data/slam/kitti/odometry/sequences/<id>/` |
| TUM RGB-D | `/data/slam/TUM/rgbd_dataset_freiburg1_desk/` |
| TUM-VI | `/data/slam/TUM-VI/dataset-room1_512_16/` |
| Oxford Spires | `/data/Oxford/sequences/<seq>/raw/{imu.csv,lidar-clouds/}` |

Spires 默认包是 IMU + Velodyne zip + 真值，不含相机。加 `--spires-images` 才下 `images.zip`。

## 下载

在**可写**的宿主机盘上下，再挂进容器：

```bash
export AUTONOMY_DATASETS=/path/to/datasets
src/autonomy/autonomy/localization/scripts/download_datasets.sh
```

容器内仅当 `/data` 可写时：

```bash
src/autonomy/autonomy/localization/scripts/download_datasets.sh --dest /data
```

查看全部组名和序列名：

```bash
src/autonomy/autonomy/localization/scripts/download_datasets.sh --list
```

只下某一组或某几条（只写了序列参数时，不会再拉其它组）：

```bash
src/autonomy/autonomy/localization/scripts/download_datasets.sh --datasets euroc,kitti
src/autonomy/autonomy/localization/scripts/download_datasets.sh --euroc MH_01,V1_01
src/autonomy/autonomy/localization/scripts/download_datasets.sh \
  --spires keble-college-02 --tum-vi room1 --tum-rgbd freiburg1_desk
src/autonomy/autonomy/localization/scripts/download_datasets.sh \
  --datasets euroc,spires --euroc MH_04 --spires christ-church-01
src/autonomy/autonomy/localization/scripts/download_datasets.sh --lite
```

| 参数 | 作用 |
|------|------|
| （默认，无序列标志） | EuRoC 11 条、KITTI 灰度+poses+calib、TUM RGB-D Freiburg1 等、TUM-VI room1–6、全部 Spires LIO 包 |
| `--list` | 打印目录后退出 |
| `--lite` | MH_01、KITTI 三件套、`freiburg1_desk`、TUM-VI room1、Spires `keble-college-02` |
| `--datasets euroc,kitti` | 组：`euroc,kitti,tum-rgbd,tum-vi,spires` |
| `--euroc MH_01,V1_01` | EuRoC（可用短名） |
| `--tum-rgbd freiburg1_desk` | TUM RGB-D |
| `--tum-vi room1,corridor1` | TUM-VI（`--tum-vi-full` 才包含走廊/大厅/室外/滑梯全集） |
| `--spires keble-college-02` | Spires（可用后缀） |
| `--kitti-velodyne` | 额外 Velodyne bins（体积很大） |
| `--spires-images` | Spires 相机 zip |
| `--dest PATH` | 根目录，默认 `$AUTONOMY_DATASETS` 或 `/data` |

KITTI 官方包是整档 zip，不能按 `00`/`01` 拆条下载。下载完成后用 `info` 检查布局，不要凭压缩包是否存在判断可用。

## 发布器

```bash
/usr/bin/python3 -m autonomy.tools.datasets --help
/usr/bin/python3 -m autonomy.tools.datasets info \
  --dataset euroc \
  --path /data/slam/EuRoC/data/MH_01_easy
```

`info` 只打印 JSON，不发话题。`publish` 必须在对应 launch **已经起来**之后执行。

| `--dataset` | 默认传感器 | Atlas / Argus |
|-------------|----------------|---------------|
| `euroc` | `monocular` | Atlas，配合 `--atlas-sensor` |
| `tum-rgbd` | `rgbd` | Atlas |
| `tum-vi` | `stereo_inertial` | Atlas（目录须含完整 `mav0/`） |
| `kitti` | `stereo` | Atlas，序列 00–02 与 `kitti_00_02.yaml` 匹配 |
| `spires` | 不适用 | Argus：`/imu`、`/points`，可选图像与真值 |

视觉发布器交错发送同步图像，并在惯性模式每个图像时刻前送齐截至该时刻的 IMU。Atlas 同步容差：双目 10 ms，RGB-D 30 ms。建议 `--rate 1.0`；`--rate 0` 会淹没队列。视觉数据不要 `--loop`。冒烟可加 `--start 0 --max-frames 100`（Spires 可用 `--max-frames 400`）。

完整话题表与各数据集复制即用的 `publish` 命令见 [18_Tools/08_datasets.md](../../../docs/source/18_Tools/08_datasets.md)。
