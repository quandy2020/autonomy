# 6. Cartographer 2D

以下命令在 SpaceHero 容器内执行。同一时刻只起一个定位后端，必须在 `/workspace/autonomy/src/autonomy` 启动 launch。

| launch | 配置 | 数据 |
|--------|------|------|
| `localization_cartographer.launch` | `backpack_2d.lua` | Museum record：`/echoes_1` + `/imu` |
| `localization_cartographer_autosim.launch` | `autosim_2d.lua` | AutoSim `/scan` + `/odom` |

Lua 细节：[Cartographer 指南](../../../docs/source/06_Localization/07_cartographer/guide.md)。

## Deutsches Museum record

转换好的文件：

`/workspace/autonomy/data/records/cartographer/cartographer_paper_deutsches_museum.record`

准备三个容器终端，**每个终端先执行一次**：

```bash
source /workspace/autonomy/install/setup.bash
cd /workspace/autonomy/src/autonomy
mkdir -p data/maps
export PATH=/workspace/autonomy/build/autonomy/bin:$PATH
export LD_LIBRARY_PATH=/workspace/autonomy/build/autonomy/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export AUTOLINK_PATH=$PWD/autolink/autolink
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
```

必须同时设置 `PATH` 和 `LD_LIBRARY_PATH`：前者选择 build 程序，后者选择同一版本的 build 动态库。只设置 `PATH` 会加载 `install/autonomy/lib` 的旧库，可能造成 TF 时间戳和位姿异常。设置环境后需重新启动旧进程才会生效。

1. 终端一启动 Cartographer（终端保持运行）：

```bash
autolink launch start localization_cartographer.launch
```

启动日志应包含 `Cartographer backend started`，配置路径应为 `/workspace/autonomy/src/autonomy/autonomy/localization/conf/cartographer/backpack_2d.lua`。从工作空间根目录启动时，相对配置路径可能回退到 install 中的旧配置。

2. 终端二等定位进程起来后再播 record（`-a` 播全部频道，不要 `--loop`）：

```bash
autolink recorder play -a -r 1.0 -f \
  /workspace/autonomy/data/records/cartographer/cartographer_paper_deutsches_museum.record
```

冒烟可把 `-r 1.0` 改为 `-r 5` 加快。完整一条约 32 分钟实时。重新回放前先停止播放器和定位进程，等旧进程完全退出，再按步骤 1、2 启动，避免时间戳回绕。

3. 终端三启动 Autoviz，Fixed Frame 为 `map`：

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
| Trajectory (`Path`) | `/trajectory` |

开头若干扫描会累积（`num_accumulated_range_data = 10`），之后才出 `/map`。博物馆尺度把视距拉到几十米。

OccupancyMap 使用 RViz ROS 的 `map` 配色和默认 Alpha `0.7`：空闲为白色、占用为黑色、未知为灰绿色 `#708986`。该灰绿色就是 RViz 的未知栅格颜色；需要查看不透明的原始调色板时，将 Alpha 改为 `1.0`。参考 [RViz 官方 MapDisplay](https://github.com/ros-visualization/rviz/blob/noetic-devel/src/rviz/default_plugin/map_display.cpp)。

`/map` 保留 Cartographer 的占用概率和原始墙体宽度，不做黑白二值化、墙体膨胀或未知边界补墙。中间概率显示为灰度；修复前运行的节点需要重新启动才能使用此逻辑。

需要像 Cartographer RViz 插件一样查看独立子图时：

1. 取消勾选 `OccupancyMap`，勾选配置中新增的 `Submaps`。
2. 保持 Channel 为 `/submap_list`、Submap Query Service 为 `/submap_query`。
3. `Submap IDs` 显示 `(轨迹编号, 子图编号)`；`Submap Pose Axes` 显示每个子图的坐标轴。
4. `Trajectory ID` 为 `-1` 时显示全部轨迹，改成具体编号可单独查看。
5. 二维地图使用 `High Resolution`；三维子图还可选择 `Low Resolution`。
6. `Tracking Frame` 默认为 `base_link`。`Fade-out Distance (m)` 默认为 `1`：高度差超过 1 米开始渐隐，达到 3 米完全隐藏；设为 `0` 可关闭渐隐。

子图纹理按版本缓存，回环优化只改变位姿时不会重复下载。此显示项随 autonomy 集成构建提供；独立 Autoviz 未包含 Cartographer 消息时不注册该项。

此配置的 Orbit 视角跟随 `base_link`，Global Fixed Frame 仍为 `map`。已经打开的窗口可在右侧 Views 中把 Target Frame 设为 `base_link`，再将 Focal Point 的 X/Y/Z 都设为 `0`、Distance 设为 `40`、Pitch 设为 `1.37`，让相机围绕当前机器人观察。激光保持真实三维姿态，不强行压到地图的 Z=0 平面；状态为 Ok 但看不到时，先检查视野是否仍停在原点。

若显示 `TF missing: horizontal_laser_link -> map`，另开一个终端执行公共环境后检查：

```bash
timeout 15 autolink channel echo /tf_static --once
timeout 15 autolink channel echo /tf --once
```

`/tf_static` 应含 `base_link → imu_link / horizontal_laser_link / vertical_laser_link`；`/tf` 应含动态的 `map → odom → base_link`。Museum 回放的动态 TF 时间戳应在 2015 年，与传感器数据一致，不能外推到当前年份。仅看到 `/tf_static` 的发布日志，还不能证明整条变换链可用。

4. 停止：Ctrl-C 停播放器，再：

```bash
autolink launch stop localization_cartographer.launch
```

状态写到启动目录下的 `data/maps/cartographer.pbstream`，即 `/workspace/autonomy/src/autonomy/data/maps/cartographer.pbstream`。

## AutoSim / Habitat

```bash
autolink launch start localization_cartographer_autosim.launch
```

仿真用带 `/scan` 的 slam 配置。或 `scripts/start_cartographer.sh`（现在起 backpack launch）。
