# 7. Gazebo 与 ROS 集成

需要 Gazebo 物理与 `/scan`、相机，且接受 ROS 2 依赖。

外部包 **autonomy_ros**、**autonomy_gazebo**（不在本仓库）：Gazebo ⇄ bridge ⇄ Autonomy C++ 栈。

| 对比 | 多进程+Bridge | Gazebo+ROS |
|------|---------------|------------|
| 依赖 | 轻 | ROS2+Gazebo |
| 传感器 | 注入/简单 | 插件丰富 |

### 启动示例

```bash
export GLOG_logtostderr=1
source /opt/ros/humble/setup.bash
source /workspace/autonomy/install/setup.bash

ros2 launch autonomy_ros autonomy.launch.py \
  use_gazebo:=true world_type:=house use_sim_time:=true

# 或独立 Gazebo：
ros2 launch autonomy_gazebo empty_world.launch.py gui:=true spawn_robot:=true
```

Docker ROS 依赖：`docker/install/install_ros2.sh`。

### Launch 参数（常见）

| 参数 | 说明 |
|------|------|
| `use_sim_time` | 订阅 `/clock` |
| `use_gazebo` | 启 Gazebo |
| `world_type` | house / world 等 |
| `x_pose`, `y_pose` | 初始位姿 |

### 话题

| Topic | 方向 | 用途 |
|-------|------|------|
| `/cmd_vel` | → Gazebo | 控制 |
| `/odom`, `/scan` | ← | 定位、costmap |
| `/clock` | ← | 仿真时间 |
| TF map→odom→base_link | 双向 | 与 URDF 一致 |

### 常见坑

| 现象 | 处理 |
|------|------|
| 不动 | `/cmd_vel`、bridge |
| TF 乱 | `use_sim_time` + `/clock` |

### 延伸阅读

[04 Running · ROS2](../04_Running/06_ros2_integration.md) · [10 Perception · 管线](../10_Perception/06_sensor_pipeline.md)
