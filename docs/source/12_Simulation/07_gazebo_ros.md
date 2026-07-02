# 7. Gazebo 与 ROS 集成

本文描述通过外部包 `autonomy_ros` 与 `autonomy_gazebo` 进行物理仿真与传感器仿真的集成方式。

> 这些包**不在**本 `autonomy` 仓库内，但为官方推荐的完整仿真路径。

---

## 7.1 架构

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│   Gazebo    │────→│ autonomy_ros │────→│  Autonomy   │
│  (physics)  │←────│   (bridge)   │←────│  (C++ 栈)   │
└─────────────┘     └──────────────┘     └─────────────┘
     │                      │
  /scan, /odom          cmd_vel, TF
  /camera, /clock
```

---

## 7.2 环境准备

```bash
export GLOG_logtostderr=1
export AUTOLINK_PATH=/workspace/autonomy/src/autonomy/autolink/autolink
export AUTONOMY_MODEL=waffle

source /opt/ros/humble/setup.bash
source /workspace/autonomy/install/setup.bash
```

Docker 安装 Gazebo ROS 包见 `docker/install/install_ros2.sh`（`gazebo-ros`, `turtlebot3-gazebo` 等）。

---

## 7.3 启动命令

### 7.3.1 一体化启动

```bash
ros2 launch autonomy_ros autonomy.launch.py \
  use_gazebo:=true \
  world_type:=house \
  use_sim_time:=true
```

### 7.3.2 独立 Gazebo

```bash
ros2 launch autonomy_gazebo empty_world.launch.py gui:=true spawn_robot:=true
ros2 launch autonomy_gazebo autonomy_house.launch.py gui:=true spawn_robot:=true
```

---

## 7.4 Launch 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `use_sim_time` | true | 订阅 `/clock`，与 Gazebo 同步 |
| `use_gazebo` | false | 是否 spawn Gazebo 世界 |
| `world_type` | house | `house` / `world` |
| `x_pose`, `y_pose` | -2.0, -0.5 | 机器人初始位姿 |
| `gui` | true | Gazebo GUI |
| `spawn_robot` | true | 是否生成机器人模型 |

---

## 7.5 Topic 与数据流

| Topic | 方向 | 说明 |
|-------|------|------|
| `/cmd_vel` | Autonomy → Gazebo | 速度控制 |
| `/odom` | Gazebo → Autonomy | 里程计 |
| `/scan` | Gazebo → Autonomy | 激光，注入 costmap |
| `/clock` | Gazebo → ROS | 仿真时间 |
| TF `map→odom→base_link` | 双向 | 位姿变换 |

`config/planner/planner.lua` 注释：激光经 `autonomy_ros /scan` bridge 注入 obstacle_layer。

---

## 7.6 costmap 与仿真对齐

- `base_footprint` / `base_link` 与 Gazebo 机器人 URDF 一致
- `global_frame` 通常为 `map`
- `use_sim_time:=true` 时，所有节点使用仿真时钟

---

## 7.7 与 nav_test 的对比

| 维度 | nav_test | Gazebo + ROS |
|------|----------|--------------|
| 依赖 | 无 ROS | ROS2 + Gazebo |
| 物理 | 运动学积分 | 物理引擎 |
| 传感器 | 无 | 激光、相机、IMU |
| 动态障碍 | 无 | 可扩展 |
| CI 友好 | ✅ | 较重 |
| 调试可视化 | 日志 | RViz + Gazebo GUI |

---

## 7.8 故障排查

| 现象 | 处理 |
|------|------|
| 机器人不动 | 检查 `/cmd_vel` 是否发布；Gazebo 插件是否加载 |
| 无激光 | 确认 `/scan` topic；bridge 配置 |
| TF 断裂 | `ros2 run tf2_tools view_frames` 检查树 |
| 时间不同步 | 确保 `use_sim_time:=true` 且 `/clock` 发布 |

---

## 7.9 延伸阅读

- [04 Running · ROS 2 集成](../04_Running/06_ros2_integration.md)
- [10 Perception · 传感器管线](../10_Perception/06_sensor_pipeline.md)
