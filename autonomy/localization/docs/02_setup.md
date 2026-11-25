# 2. 编译与公共环境

以下命令默认在 `SpaceHero` 容器中执行。

## 编译

```bash
docker exec -it SpaceHero /bin/bash
cd /workspace/autonomy
colcon build --packages-up-to autonomy
```

产物：`build/autonomy/bin/autonomy.localization`、`autoviz`，以及 Python 包 `autonomy.tools.datasets`。

## 每个终端都要设的环境

```bash
source /workspace/autonomy/install/setup.bash
cd /workspace/autonomy/src/autonomy
export PATH=/workspace/autonomy/build/autonomy/bin:$PATH
export PYTHONPATH="/workspace/autonomy/src/autonomy:/workspace/autonomy/install/autonomy/python:/workspace/autonomy/install/autonomy/lib/python:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="/workspace/autonomy/install/autonomy/lib:/workspace/autonomy/build/autonomy/lib:${LD_LIBRARY_PATH:-}"
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
```

Isaac 镜像用 `/usr/bin/python3`，不要依赖 `python` alias。

启动前确认没有其它定位进程：

```bash
pgrep -af autonomy.localization
autolink launch list
```

## 数据盘挂载

`run_autonomy.py` 把宿主机数据集根挂到容器 `/data`。改挂载后必须重建容器：

```bash
export AUTONOMY_DATASETS=/path/to/datasets
AUTONOMY_FORCE_RECREATE=1 python3 docker/run_autonomy.py
```

语法为 `HOST` 或 `HOST:CONTAINER[:MODE]`。CLI `--datasets` 优先于环境变量。`/data` 常为只读：下载和解压在宿主机完成，不要假定容器内能写 `/data`。

## 直接跑二进制

一般用 launch。需要改话题时：

```bash
autonomy.localization \
  --localization_mode=atlas \
  --atlas_sensor=stereo_inertial \
  --atlas_config=autonomy/localization/atlas/config/euroc_stereo_inertial.yaml \
  --atlas_vocabulary=autonomy/localization/atlas/config/orb_vocab.fbow \
  --atlas_image_topic=/camera/rgb/image_raw \
  --atlas_right_image_topic=/camera/right/image_raw \
  --atlas_imu_topic=/imu
```

`--localization_mode` 取值：`cartographer` | `atlas` | `argus`。完整选项见 `autonomy.localization --help`。
