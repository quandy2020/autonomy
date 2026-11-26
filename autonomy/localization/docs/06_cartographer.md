# 6. Cartographer 2D

同一时刻只起一个定位后端。必须在 `/workspace/autonomy/src/autonomy` 启动 launch。

| launch | 配置 | 数据 |
|--------|------|------|
| `localization_cartographer.launch` | `backpack_2d.lua` | Museum record：`echoes_1` + `imu` |
| `localization_cartographer_autosim.launch` | `autosim_2d.lua` | AutoSim `/scan` + `/odom` |

Lua 细节：[Cartographer 指南](../../../docs/source/06_Localization/07_cartographer/guide.md)。

## Deutsches Museum record

转换好的文件：

`data/records/cartographer/cartographer_paper_deutsches_museum.record`

公共环境：

```bash
source /workspace/autonomy/install/setup.bash
cd /workspace/autonomy/src/autonomy
mkdir -p data/maps
export PATH=/workspace/autonomy/build/autonomy/bin:$PATH
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
```

1. 启动 Cartographer：

```bash
autolink launch start localization_cartographer.launch
```

2. 等进程起来后再播 record（`-a` 播全部频道，不要 `--loop`）：

```bash
autolink recorder play -a -r 1.0 -f \
  data/records/cartographer/cartographer_paper_deutsches_museum.record
```

冒烟可加 `-r 5` 加快。完整条约 32 分钟实时。再播一次前先 `launch stop` 再 start，避免时间戳回绕。

3. Autoviz，Fixed Frame 为 `map`：

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

开头若干扫描会累积（`num_accumulated_range_data = 10`），之后才出 `/map`。博物馆尺度把视距拉到几十米。

4. 停止：Ctrl-C 停播放器，再：

```bash
autolink launch stop localization_cartographer.launch
```

状态写到 `data/maps/cartographer.pbstream`。

## AutoSim / Habitat

```bash
autolink launch start localization_cartographer_autosim.launch
```

仿真用带 `/scan` 的 slam 配置。或 `scripts/start_cartographer.sh`（现在起 backpack launch）。
