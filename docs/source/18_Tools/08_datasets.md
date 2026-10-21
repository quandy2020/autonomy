# 8. 数据集发布

实现：`autonomy/tools/datasets`，入口 `python -m autonomy.tools.datasets`。

**什么时候用：** Oxford Spires 等离线序列要在容器里喂给 Argus / localization，而不想录 live bag。

### 话题约定

与 `localization_argus.launch` / autosim 一致：

| 话题 | 消息 | 源 |
|------|------|-----|
| `/imu` | Imu | `imu.csv` |
| `/points` | PointCloud2 | `lidar-clouds/*.pcd` |
| `/camera/rgb/image_raw` | Image | 相机目录（`--no-image` 可关） |
| `/gt/odometry` | Odometry | TUM 真值（跟雷达时刻；`--no-gt` 关闭） |
| `/gt/path` | Path | 每帧追加一个 GT 位姿（绝对 TUM/`map`）；Argus 用首帧 `/gt/odometry` 做 SE(3) 对齐到同一坐标系（`--gt-zero-start` 可改为相对原点） |

### 宿主机挂载

`run_autonomy.py` 将数据集根挂到容器 **`/data`（默认 ro）**。

```bash
export AUTONOMY_DATASETS=/path/to/datasets
AUTONOMY_FORCE_RECREATE=1 python3 docker/run_autonomy.py
```

语法：`HOST` 或 `HOST:CONTAINER[:MODE]`；CLI `--datasets` 优先于环境变量。

### 容器内 Python

Isaac 镜像用 `/usr/bin/python3`，不要用 alias 的 `python`。`install/autonomy/python`（autolink）须在 `lib/python`（automsgs）**之前**，并带上 `LD_LIBRARY_PATH`（否则缺 `libautolink.so` / `automsgs.msgs`）：

```bash
export LD_LIBRARY_PATH="/workspace/autonomy/install/autonomy/lib:/workspace/autonomy/build/autonomy/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="/workspace/autonomy/src/autonomy:/workspace/autonomy/install/autonomy/python:/workspace/autonomy/install/autonomy/lib/python:${PYTHONPATH:-}"
```

### Oxford Spires 示例

```bash
# A: localization
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
autolink launch start localization_argus.launch

# B: publish（勿粘贴带尖括号的占位路径）
export LD_LIBRARY_PATH="/workspace/autonomy/install/autonomy/lib:/workspace/autonomy/build/autonomy/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="/workspace/autonomy/src/autonomy:/workspace/autonomy/install/autonomy/python:/workspace/autonomy/install/autonomy/lib/python:${PYTHONPATH:-}"
# 默认 = 图像 + 真值轨迹（勿加 --no-image / --no-gt）
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset spires \
  --path /data/Oxford/sequences/2024-03-12-keble-college-02 \
  --rate 1.0
```

Autoviz 用预设一次订齐 Argus + 数据集话题：

```bash
./build/autonomy/bin/autoviz --config src/autonomy/autoviz/config/argus.autoviz
```

含 `/camera/rgb/image_raw`、`/points`、`/argus/trajectory`（绿）、`/gt/path`（黄，**随播放拉长**，不是一次画完全程）等。

`info` 子命令只看帧数/时长，不发布。

### 常见坑

| 现象 | 说明 |
|------|------|
| 无点云 | path 要到 `raw` 或序列根；看 `info` |
| 无图像 | 不要加 `--no-image`；确认 `raw/cam0` |
| 无 GT 轨迹 | 不要加 `--no-gt`；订 `/gt/path`；文件在 `processed/vilens-slam/gt.tum` |
| **无全局点云** | 全局图是 **`/argus/cloud_map`（Argus 发）**。datasets 重播时间回绕后旧逻辑会停发（已修）；重启 Argus，`autolink channel hz /argus/cloud_map` 应约 1 Hz。Autoviz Fixed Frame=`map`，色变换用 **RGB8** |
| Image 黑屏但 Properties 已有 Channel | Channel 下拉未真正应用（已修）；重载 `argus.autoviz`，或重新点选 Channel |
| Argus 无输出 / `/argus/trajectory` 静默 | 数据集重播会 `loop back`；需重启 Argus（已修：自动 Reset Sync/IMU）。看 `autolink channel hz /argus/odometry` |
| 播放比真值慢 / 进度帧号乱跳 | `--rate 1` 按传感器时间轴调度（已扣编码耗时）；进度每 10 帧打一行。赶不及会追赶、不额外堆 sleep |
| 改挂载不生效 | 需 `AUTONOMY_FORCE_RECREATE=1` |

参见 [06 Localization](../06_Localization/02_quickstart.md)、[04 Running · Docker](../04_Running/05_docker_runtime.md)。
