# 7. Gazebo 与 ROS 集成

外部包：`autonomy_ros`、`autonomy_gazebo`（不在本仓库）。

### 7.1 架构

```
Gazebo ⇄ autonomy_ros (bridge) ⇄ Autonomy C++ 栈
/scan,/odom,/camera,/clock · cmd_vel, TF
```

### 7.2 环境与启动

```bash
export GLOG_logtostderr=1
export AUTOLINK_PATH=/workspace/autonomy/src/autonomy/autolink/autolink
export AUTONOMY_MODEL=waffle
source /opt/ros/humble/setup.bash
source /workspace/autonomy/install/setup.bash

ros2 launch autonomy_ros autonomy.launch.py \
use_gazebo:=true world_type:=house use_sim_time:=true

# 或独立 Gazebo：
ros2 launch autonomy_gazebo empty_world.launch.py gui:=true spawn_robot:=true
ros2 launch autonomy_gazebo autonomy_house.launch.py gui:=true spawn_robot:=true
```

Docker 依赖见 `docker/install/install_ros2.sh`。

### 7.3 Launch 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `use_sim_time` | true | 订阅 `/clock` |
| `use_gazebo` | false | 启 Gazebo |
| `world_type` | house | house / world |
| `x_pose`,`y_pose` | -2.0,-0.5 | 初始位姿 |
| `gui` / `spawn_robot` | true | GUI / 生成模型 |

### 7.4 Topic

| Topic | 方向 | 说明 |
|-------|------|------|
| `/cmd_vel` | → Gazebo | 速度 |
| `/odom`,`/scan` | ← Gazebo | 里程计、激光→costmap |
| `/clock` | ← Gazebo | 仿真时间 |
| TF `map→odom→base_link` | 双向 | 位姿 |

对齐：`base_link` 与 URDF 一致；`global_frame=map`；`use_sim_time:=true`。

### 7.5 对比与排错

| 维度 | 多进程（无 Gazebo） | Gazebo+ROS |
|------|---------------------|------------|
| 依赖 | Autolink | ROS2+Gazebo |
| 传感器 | Bridge 注入 | 插件丰富 |
| CI | 较轻 | 较重 |

| 现象 | 处理 |
|------|------|
| 不动 / 无激光 | 查 `/cmd_vel`、`/scan`、bridge |
| TF / 时间 | `view_frames`；`use_sim_time` + `/clock` |

### 7.6 参见

- [04 Running · ROS2](../04_Running/06_ros2_integration.md) · [10 Perception · 管线](../10_Perception/06_sensor_pipeline.md)
