# 2. 快速开始

最快看到地图/路径/激光，按你已有的工具链三选一。

Autonomy 业务 bin **不带 GUI**；必须额外起 Autoviz、Foxglove bridge+Studio，或 RViz2。

| 条件 | 工具 |
|------|------|
| 无 ROS | Autoviz |
| 浏览器 | Foxglove |
| 习惯 RViz | RViz2 |

### RViz2（已有 ROS 2）

```bash
source /opt/ros/humble/setup.bash
source /path/to/autonomy/install/setup.bash
ros2 launch autonomy_ros autonomy.launch.py
rviz2
```

| Display | 常见 Topic |
|---------|------------|
| Map / Path / LaserScan | `/map`、`/plan` 或 `/planning/path`、`/scan` |
| TF | `map` → `base_link` |

详见 [§7](07_rviz2_ros2.md)。

### Foxglove

```bash
ros2 launch autonomy_ros autonomy.launch.py
ros2 run foxglove_bridge foxglove_bridge
# Studio → ws://localhost:8765
```

Docker 映射 8765。详见 [§6](06_foxglove.md)。

### 纯 C++ 栈（无 GUI）

```bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
autolink_launch autonomy.launch
```

要看画面需另起 [Autoviz](09_autoviz.md) 或 ROS 桥。

### 验证

```bash
ros2 topic list
ros2 topic echo /plan
ros2 topic hz /scan
```

### 常见坑

| 现象 | 说明 |
|------|------|
| 无 VisualizationServer 进程 | 未实现，正常 |
| 纯 launch 无 ROS topic | 未起 autonomy_ros |

### 延伸阅读

[00 指南](00_guide.md)
