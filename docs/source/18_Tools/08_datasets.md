# 8. 数据集发布

将离线传感器序列经 autolink 按时间序发布，供 Argus 等节点订阅。实现：`autonomy/tools/datasets`，入口：`python -m autonomy.tools.datasets`。

### 9.1 话题约定

与 `localization_argus.launch` / autosim 一致：

| 话题 | 消息 | 源 |
|------|------|-----|
| `/imu` | `Imu` | `imu.csv` |
| `/points` | `PointCloud2` | `lidar-clouds/*.pcd` |
| `/camera/rgb/image_raw` | `Image` | 相机目录（`--no-image` 可关） |

### 9.2 挂载（宿主机）

`run_autonomy.py` 将宿主机数据集根挂到容器 **`/data`（默认只读）**。

| 输入 | 语法 |
|------|------|
| 环境变量 | `AUTONOMY_DATASETS=HOST` 或 `HOST:CONTAINER[:MODE]` |
| CLI | `--datasets`（同上；优先于环境变量） |

仅写 `HOST` 时等价于 `HOST:/data:ro`。与 `AUTONOMY_DATA_VOLUMES`（通用盘）独立。

```bash
export AUTONOMY_DATASETS=/path/to/datasets # 内含 Oxford/ 等
AUTONOMY_FORCE_RECREATE=1 python3 docker/run_autonomy.py
```

变更挂载须重建容器。成功后容器内 `AUTONOMY_DATASETS=/data`。

### 9.3 运行环境（容器）

Isaac 镜像勿用被 alias 的 `python`。使用系统解释器，且 **`install/autonomy/python` 须在 `lib/python` 之前**（否则缺少 `autolink.Node`）：

```bash
source /workspace/autonomy/install/setup.bash
export PYTHONPATH="/workspace/autonomy/src/autonomy:/workspace/autonomy/install/autonomy/python:/workspace/autonomy/install/autonomy/lib/python${PYTHONPATH:+:$PYTHONPATH}"
```

### 9.4 Oxford Spires

布局：

```text
sequences/<name>/raw/{imu.csv, lidar-clouds/, cam0|cam1|cam2/}
```

`--path`：序列根或 `.../raw`。

```bash
# 终端 A
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
autolink launch start localization_argus.launch

# 终端 B
/usr/bin/python3 -m autonomy.tools.datasets publish \
--dataset spires \
--path /data/Oxford/sequences/<name> \
--rate 1.0
```

| 选项 | 默认 | 含义 |
|------|------|------|
| `--rate` | `1.0` | 相对数据集时间倍速；`0` 尽快 |
| `--no-image` | 关 | 只发 IMU + 点云 |
| `--start` / `--max-frames` | `0` / 全长 | 截取 |
| `--camera` | `cam0` | 相机子目录 |

`info` 子命令打印帧数与时长，不发布。

### 9.5 参见

- [06 Localization · §2.4](../06_Localization/02_quickstart.md)
- [04 Running · Docker](../04_Running/05_docker_runtime.md)
