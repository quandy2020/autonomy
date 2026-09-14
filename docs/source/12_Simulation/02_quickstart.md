# 2. 快速开始

### 2.1 最快路径：多进程栈（无 ROS）

进程内 `autonomy_nav_test` **已移除**。本地验证用 launch 拉起 planning / control / task 等，再用 Bridge 或 Action 发令：

```bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
export AUTONOMY_BT_PLUGIN_PATH="$PWD/build/lib"
export GLOG_logtostderr=1

autolink_launch autonomy.launch
```

详见 [04 Running · 快速运行](../04_Running/02_quickstart.md)。

### 2.2 Gazebo 仿真（ROS2）

```bash
# 环境
source /opt/ros/humble/setup.bash
source /workspace/autonomy/install/setup.bash

# 启动 Gazebo + Autonomy
ros2 launch autonomy_ros autonomy.launch.py \
  use_gazebo:=true \
  world_type:=house \
  use_sim_time:=true
```

详见 [§6 Gazebo ROS](07_gazebo_ros.md)。

### 2.3 Docker 快速上手

```bash
python3 docker/run_autonomy.py -p x86_64

# 容器内编译后
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
autolink_launch autonomy.launch
```

### 2.4 工具选型

| 需求 | 工具 |
|------|------|
| 验证 BT + 全栈导航 | `autolink_launch` + Bridge / Action |
| 物理 + 传感器 | Gazebo + `autonomy_ros` |
| 车辆限幅 / Stage | [§7 Vehicle Stage](08_vehicle_stage.md) |

### 2.5 环境变量

| 变量 | 说明 |
|------|------|
| `AUTOLINK_LAUNCH_PATH` | launch 文件搜索路径 |
| `AUTONOMY_BT_PLUGIN_PATH` | BT 插件 `.so` 搜索路径 |
| `GLOG_logtostderr=1` | 日志输出到终端 |
| `AUTOLINK_PATH` | Autolink 配置路径（ROS 模式） |
