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

### 宿主机挂载

`run_autonomy.py` 将数据集根挂到容器 **`/data`（默认 ro）**。

```bash
export AUTONOMY_DATASETS=/path/to/datasets
AUTONOMY_FORCE_RECREATE=1 python3 docker/run_autonomy.py
```

语法：`HOST` 或 `HOST:CONTAINER[:MODE]`；CLI `--datasets` 优先于环境变量。

### 容器内 Python

Isaac 镜像避免 alias 的 `python`。`install/autonomy/python` 须在 `lib/python` **之前**（否则缺 `autolink.Node`）：

```bash
source /workspace/autonomy/install/setup.bash
export PYTHONPATH="/workspace/autonomy/src/autonomy:/workspace/autonomy/install/autonomy/python:..."
```

### Oxford Spires 示例

```bash
# A: localization
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
autolink launch start localization_argus.launch

# B: publish
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset spires --path /data/Oxford/sequences/<name>/raw --rate 1.0
```

`info` 子命令只看帧数/时长，不发布。

### 常见坑

| 现象 | 说明 |
|------|------|
| 无点云 | path 要到 `raw` 或序列根；看 `info` |
| Argus 无输出 | launch 与话题名不一致 |
| 改挂载不生效 | 需 `AUTONOMY_FORCE_RECREATE=1` |

参见 [06 Localization](../06_Localization/02_quickstart.md)、[04 Running · Docker](../04_Running/05_docker_runtime.md)。
