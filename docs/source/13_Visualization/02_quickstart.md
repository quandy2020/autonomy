# 2. 快速开始

### 2.1 RViz2

```bash
source /opt/ros/humble/setup.bash
source /path/to/autonomy/install/setup.bash
ros2 launch autonomy_ros autonomy.launch.py
rviz2
```

| Display | Topic / Frame |
|---------|---------------|
| Map / Path / LaserScan | `/map`、`/plan` 或 `/planning/path`、`/scan` |
| TF | `map` → `base_link` |

详见 [§7](07_rviz2_ros2.md)。

### 2.2 Foxglove

```bash
ros2 launch autonomy_ros autonomy.launch.py
ros2 run foxglove_bridge foxglove_bridge
# Studio: Open connection → ws://localhost:8765
```

Docker 映射 8765（`docker_utils.py`）。详见 [§6](06_foxglove.md)。

### 2.3 纯 C++（无 GUI）

```bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
export GLOG_logtostderr=1
autolink_launch autonomy.launch
```

### 2.4 验证

```bash
ros2 topic list
ros2 topic echo /plan
ros2 topic echo /map --once
```

### 2.5 参见
