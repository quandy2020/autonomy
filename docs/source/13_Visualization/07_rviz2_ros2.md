# 7. RViz2 与 ROS 2

RViz2 是 ROS 2 生态中最常用的 3D 可视化工具。Autonomy 通过**外部 ROS 2 包装层**与之对接。

### 7.1 架构

```
libautonomy（commsgs C++ struct）
        │
        ▼
autonomy_ros（外部包，本仓库无源码）
        │  commsgs ↔ rosidl 转换
        ▼
ROS 2 DDS topics
        │
        ▼
rviz2 / ros2 topic echo
```

`libautonomy` **不依赖 rclcpp**；Framework 本体可在无 ROS 环境下运行。

### 7.2 启动流程

```bash
source /opt/ros/humble/setup.bash
source /path/to/autonomy/install/setup.bash

ros2 launch autonomy_ros autonomy.launch.py
rviz2
```

仿真场景：

```bash
ros2 launch autonomy_ros autonomy.launch.py use_gazebo:=true world_type:=house
```

详见 [04 Running · ROS 2](../04_Running/06_ros2_integration.md)。

### 7.3 常用话题

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `/map` | `nav_msgs/OccupancyGrid` | 静态地图 |
| `/plan` | `nav_msgs/Path` | 规划路径 |
| `/scan` | `sensor_msgs/LaserScan` | 激光 |
| `/odom` | `nav_msgs/Odometry` | 里程计 |
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令 |
| `/tf` | `tf2_msgs/TFMessage` | 坐标变换 |

实际话题名以 `autonomy_ros` launch 与 `config/common.lua` 为准。

### 7.4 RViz2 显示配置

| Display 类型 | 设置 |
|--------------|------|
| **Map** | Topic: `/map`；Color Scheme: map |
| **Path** | Topic: `/plan` |
| **LaserScan** | Topic: `/scan` |
| **TF** | 显示 `map` → `base_link` |
| **RobotModel** | 若有 URDF |

初始位姿：使用 **2D Pose Estimate**（对应 AMCL 先验），见 [06 Localization · AMCL](../06_Localization/07_amcl.md)。

### 7.5 命令行验证

```bash
ros2 topic list
ros2 topic echo /plan
ros2 topic hz /scan
ros2 run tf2_tools view_frames
```

### 7.6 Docker 内使用

```bash
docker exec -it SpaceHero /bin/bash
source /opt/ros/humble/setup.bash
ros2 launch autonomy_ros autonomy.launch.py
```

### 7.7 与纯 C++ 路径对比

| 场景 | 推荐 |
|------|------|
| 算法 CI / 无 GUI | `autonomy_nav_test` |
| 交互调试 / 演示 | ROS 2 + RViz2 |
| 远程团队查看 | Foxglove（§6） |

### 7.8 相关文档

- [04 Running · 运行验证](../04_Running/07_verification.md)
- [01 Instructions · 生态](../01_Instructions/06_ecosystem.md)
