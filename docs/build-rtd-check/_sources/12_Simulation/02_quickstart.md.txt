# 2. 快速开始

### 2.1 最快路径：nav_test（无 ROS）

**三步运行**：

1. 构建：`cmake .. -DBUILD_TOOLS=ON -DBUILD_TASKS=ON && ninja autonomy_nav_test`
2. 设置 BT 插件路径：`export AUTONOMY_BT_PLUGIN_PATH=<build>/lib`
3. 运行：

```bash
./bin/autonomy_nav_test \
  --configuration_directory=/workspace/autonomy/src/autonomy/config \
  --start_x=1 --start_y=1 --start_yaw=0 \
  --goal_x=5 --goal_y=5 --goal_yaw=0 \
  --use_bt=true
```

成功输出：`Navigation succeeded.`

详见 [§6 nav_test](06_nav_test.md)。

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

详见 [§7 Gazebo ROS](07_gazebo_ros.md)。

### 2.3 Docker 快速上手

```bash
python3 src/autonomy/docker/run_autonomy.py -p x86_64

# 容器内
cmake -S src/autonomy -B build -DBUILD_TOOLS=ON -DBUILD_TASKS=ON
cmake --build build -j$(nproc)
export AUTONOMY_BT_PLUGIN_PATH=/workspace/autonomy/build/lib
./build/bin/autonomy_nav_test \
  --configuration_directory=/workspace/autonomy/src/autonomy/config \
  --start_x=1 --start_y=1 --goal_x=5 --goal_y=5
```

### 2.4 工具选型

| 需求 | 工具 |
|------|------|
| 验证 BT + 全栈导航 | `autonomy_nav_test` |
| 仅规划可视化 | `autonomy_planning_test` |
| 仅控制器跟踪 | `autonomy_controller_test` |
| 物理 + 传感器 | Gazebo + `autonomy_ros` |

### 2.5 环境变量

| 变量 | 说明 |
|------|------|
| `AUTONOMY_BT_PLUGIN_PATH` | BT 插件 `.so` 搜索路径 |
| `GLOG_logtostderr=1` | 日志输出到终端 |
| `AUTOLINK_PATH` | Autolink 配置路径（ROS 模式） |
