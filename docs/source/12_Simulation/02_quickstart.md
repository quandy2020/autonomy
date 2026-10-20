# 2. 快速开始

### 2.1 多进程栈（无 ROS）

```bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
export AUTONOMY_BT_PLUGIN_PATH="$PWD/build/lib"
export GLOG_logtostderr=1
autolink_launch autonomy.launch
```

详见 [04 Running](../04_Running/02_quickstart.md)。

### 2.2 Gazebo（ROS2）

```bash
source /opt/ros/humble/setup.bash
source /workspace/autonomy/install/setup.bash
ros2 launch autonomy_ros autonomy.launch.py \
use_gazebo:=true world_type:=house use_sim_time:=true
```

详见 [§7](07_gazebo_ros.md)。

### 2.3 Docker

```bash
python3 docker/run_autonomy.py -p x86_64
# 容器内编译后
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
autolink_launch autonomy.launch
```

### 2.4 选型与环境变量

| 需求 | 工具 |
|------|------|
| BT + 全栈 | `autolink_launch` + Bridge/Action |
| 物理 + 传感器 | Gazebo + `autonomy_ros` |
| 限幅 / Stage | [§8](08_vehicle_stage.md) |

| 变量 | 说明 |
|------|------|
| `AUTOLINK_LAUNCH_PATH` | launch 搜索路径 |
| `AUTONOMY_BT_PLUGIN_PATH` | BT 插件 `.so` |
| `GLOG_logtostderr=1` | 日志到终端 |

### 2.5 参见

- [§4](04_usage.md) · [§7](07_gazebo_ros.md) · [04 Running](../04_Running/05_docker_runtime.md)
