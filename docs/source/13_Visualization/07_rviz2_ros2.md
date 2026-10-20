# 7. RViz2 与 ROS 2

栈已通过 **autonomy_ros** 暴露 ROS 话题，要用 RViz2 看地图/路径/激光。

`libautonomy` 不依赖 rclcpp；显示走外部 **autonomy_ros** 转换后标准 ROS 2 topic。

| 场景 | 建议 |
|------|------|
| 无 ROS 栈 | Autoviz [§9](09_autoviz.md) |
| 远程 | Foxglove [§6](06_foxglove.md) |
| CI 无 GUI | 日志 + topic echo |

### 启动

```bash
source /opt/ros/humble/setup.bash
source /path/to/autonomy/install/setup.bash
ros2 launch autonomy_ros autonomy.launch.py
# 仿真：use_gazebo:=true world_type:=house
rviz2
```

见 [04 Running · ROS 2](../04_Running/06_ros2_integration.md)。

### 话题与 Display

| 话题 | 类型 | RViz Display |
|------|------|--------------|
| `/map` | OccupancyGrid | Map |
| `/plan` | Path | Path |
| `/scan` | LaserScan | LaserScan |
| `/odom` | Odometry | 可选 Odometry 插件 |
| `/cmd_vel` | Twist | 一般 Plot/echo，非 3D 必需 |
| `/tf` | TFMessage | TF |

话题名以 launch / `common.lua` 为准。初始位姿：**2D Pose Estimate** → [Localization](../06_Localization/00_guide.md)。

### 验证

```bash
ros2 topic list && ros2 topic hz /scan
ros2 run tf2_tools view_frames
```

### 常见坑

| 现象 | 说明 |
|------|------|
| 纯 autolink_launch 无 ROS topic | 未起 autonomy_ros |
| TF 断 | `use_sim_time` 与 `/clock` |

### 延伸阅读

[§2 快速开始](02_quickstart.md)
