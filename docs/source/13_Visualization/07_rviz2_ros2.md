# 7. RViz2 与 ROS 2

### 7.1 架构

```
libautonomy (commsgs) → autonomy_ros (外部) → ROS 2 topics → rviz2
```

`libautonomy` 不依赖 rclcpp。

### 7.2 启动

```bash
source /opt/ros/humble/setup.bash
source /path/to/autonomy/install/setup.bash
ros2 launch autonomy_ros autonomy.launch.py
# 仿真：use_gazebo:=true world_type:=house
rviz2
```

见 [04 Running · ROS 2](../04_Running/06_ros2_integration.md)。

### 7.3 话题与 Display

| 话题 | 类型 | Display |
|------|------|---------|
| `/map` | OccupancyGrid | Map |
| `/plan` | Path | Path |
| `/scan` | LaserScan | LaserScan |
| `/odom` | Odometry | — |
| `/cmd_vel` | Twist | — |
| `/tf` | TFMessage | TF |

话题名以 launch / `common.lua` 为准。初始位姿：RViz **2D Pose Estimate** → [Localization](../06_Localization/00_guide.md)。

### 7.4 验证与 Docker

```bash
ros2 topic list && ros2 topic echo /plan && ros2 topic hz /scan
ros2 run tf2_tools view_frames

docker exec -it SpaceHero /bin/bash
source /opt/ros/humble/setup.bash
ros2 launch autonomy_ros autonomy.launch.py
```

| 场景 | 推荐 |
|------|------|
| CI / 无 GUI | 多进程 + 日志 |
| 交互调试 | RViz2 |
| 远程 | Foxglove [§6](06_foxglove.md) |

### 7.5 参见

- [04 Running · 验证](../04_Running/07_verification.md) · [01 Instructions · 仓库](../01_Instructions/04_repository.md)
