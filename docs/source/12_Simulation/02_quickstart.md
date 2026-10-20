# 2. 快速开始

最快让栈在无真机下跑起来，或起 Gazebo 一条龙。

给出两条最常见命令路径：**纯 Autolink 多进程** 与 **ROS2+Gazebo**。

| 需求 | 工具 |
|------|------|
| BT + 全栈、少依赖 | autolink_launch + Bridge |
| 激光相机物理 | Gazebo |
| 定位联调 IMU/点云 | [AutoSim](06_autosim.md) |

### 多进程（无 ROS）

```bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
export AUTONOMY_BT_PLUGIN_PATH="$PWD/build/lib"
export GLOG_logtostderr=1
autolink_launch autonomy.launch
```

传感器需 Bridge / AutoSim / 录包注入。详见 [04 Running](../04_Running/02_quickstart.md)。

### Gazebo（ROS 2）

```bash
source /opt/ros/humble/setup.bash
source /workspace/autonomy/install/setup.bash
ros2 launch autonomy_ros autonomy.launch.py \
  use_gazebo:=true world_type:=house use_sim_time:=true
```

详见 [§7](07_gazebo_ros.md)。

### Docker

```bash
python3 docker/run_autonomy.py -p x86_64
# 容器内编译后
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
autolink_launch autonomy.launch
```

### 环境变量

| 变量 | 说明 |
|------|------|
| `AUTOLINK_LAUNCH_PATH` | launch 搜索 |
| `AUTONOMY_BT_PLUGIN_PATH` | BT 插件 |
| `GLOG_logtostderr=1` | 日志到终端 |

### 常见坑

| 现象 | 先查 |
|------|------|
| 无 `/scan` | 仿真是否起、bridge |
| BT 插件加载失败 | `AUTONOMY_BT_PLUGIN_PATH` |

### 延伸阅读

[§4](04_usage.md) · [04 Running · Docker](../04_Running/05_docker_runtime.md)
