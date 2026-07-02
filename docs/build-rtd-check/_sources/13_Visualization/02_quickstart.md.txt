# 2. 快速开始

根据你的运行环境选择一条路径。

### 2.1 路径 A：RViz2（ROS 2）

**前提**：已安装 ROS 2 Humble，且配置 `autonomy_ros` 包装层（仓库外可选包）。

```bash
source /opt/ros/humble/setup.bash
source /path/to/autonomy/install/setup.bash

ros2 launch autonomy_ros autonomy.launch.py
rviz2
```

在 RViz2 中添加：

| 显示项 | 话题 / 帧 |
|--------|-----------|
| Map | `/map` |
| Path | `/plan` 或 `/planning/path` |
| LaserScan | `/scan` |
| TF | `map` → `base_link` |

详见 [§7 RViz2](07_rviz2_ros2.md)。

### 2.2 路径 B：Foxglove Studio

1. 安装 [Foxglove Studio](https://foxglove.dev/download)（桌面或 Web）
2. 在机器人侧启动 ROS 2 与桥接：

```bash
source /opt/ros/humble/setup.bash
ros2 launch autonomy_ros autonomy.launch.py

# 另开终端
ros2 run foxglove_bridge foxglove_bridge
```

3. Foxglove 中选择 **Open connection → Foxglove WebSocket**，地址如 `ws://localhost:8765`

Docker 环境默认映射 8765 端口（见 `docker/scripts/docker_utils.py` 中 `AUTONOMY_PORTS`）。

详见 [§6 Foxglove](06_foxglove.md)。

### 2.3 路径 C：纯 C++ 离线（无 GUI）

```bash
export GLOG_logtostderr=1
./build/bin/autonomy_nav_test \
  --configuration_directory=config \
  --start_x=1 --start_y=1 \
  --goal_x=5 --goal_y=5
```

成功标志：`Navigation succeeded.` 及 `Last path poses: N`。

### 2.4 验证话题（ROS 2）

```bash
ros2 topic list
ros2 topic echo /plan
ros2 topic echo /map --once
```

### 2.5 下一步

| 目标 | 文档 |
|------|------|
| 配置 topic 列表 | [§4 配置](04_configuration.md) |
| 消息类型 | [§3 数据类型](03_data_types.md) |
| 架构理解 | [§5 架构](05_architecture.md) |
