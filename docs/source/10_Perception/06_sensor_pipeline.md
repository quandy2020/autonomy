# 6. 传感器数据管线

对接驱动或仿真，要对齐消息类型、TF 和「数据最后进 costmap 还是进 base」。

从 Driver/Sim 到 **costmap**、**localization**、**base** 的共性步骤：缓冲、坐标变换、时间对齐。

- commsgs 定义在 `autonomy/commsgs`（与 ROS 语义对齐，序列化为 Protobuf）。  
- costmap 用 `ObservationBuffer`；视觉 base 用相机话题 + 各自组件回调。  
- `PerceptionServer` 在架构图里只占「规划/占位」，不是当前主路径。

### 消息类型（高频）

| 传感器 | 消息 | 典型频率 |
|--------|------|----------|
| 2D 激光 | `LaserScan` | 10–40 Hz |
| 点云 / 深度 | `PointCloud2` | 10–20 Hz |
| RGB / 深度图 | `Image` | 15–30 Hz |
| IMU | `Imu` | 100–200 Hz |
| 里程计 | `Odometry` | 20–50 Hz |

### 数据流概览

```text
Driver / Sim → 缓冲 → TF（transform::Buffer）
    ├─ ObstacleLayer / VoxelLayer → costmap
    ├─ localization（Atlas / Argus 等）
    └─ perception/base（Image + CameraInfo）
```

### ObservationBuffer（costmap）

源码：`map/costmap_2d/observation_buffer.hpp`。

| 能力 | 说明 |
|------|------|
| 入队 / 出队 | `bufferObservation` / `getObservations` |
| 过期 | `observation_keep_time` |

### TF

```cpp
transform::Buffer::Instance()->lookupTransform(target, source, time, &tf);
// p_world = R * p_sensor + t
```

| 策略 | 场景 |
|------|------|
| 最近邻 | 低频融合 |
| 插值 | IMU 高频 + 相机低频 |
| 近似时间同步 | 多话题软对齐 |

### 激光 → costmap（可用路径）

```text
/scan → ObstacleLayer → TF → 射线清除/标记 → costmap
```

| 参数 | 含义 |
|------|------|
| `obstacle_range` | 标记最远距离 |
| `raytrace_range` | 清除最远距离 |
| `max_obstacle_height` | 高度过滤 |

### 相机 / 点云 → 视觉（Base）

```text
Image (+ CameraInfo) → base 任务 → /perception/base/...
PointCloud2 → 可进 VoxelLayer（导航），或自建感知（需自写接线）
```

### 仿真对接

| 方式 | 传感器来源 |
|------|------------|
| AutoSim / Bridge | `/imu`、`/points`、相机等，见 [12 Simulation](../12_Simulation/06_autosim.md) |
| Gazebo + autonomy_ros | `/scan`、`/camera`、`/odom` |
| 仅注入 odom/TF | 多进程 + Bridge，无物理 |

### 常见坑

| 现象 | 先查 |
|------|------|
| costmap 有、视觉无 | 相机 topic 是否接到 base DAG |
| TF 时间外推失败 | 仿真 `use_sim_time` 与 `/clock` |
| 点云不进 costmap | `data_type`、高度区间、VoxelLayer 是否启用 |

### 延伸阅读

[§3.1 数学](03_math.md) · [§8 障碍](08_obstacle_perception.md) · [12 Simulation](../12_Simulation/00_guide.md)
