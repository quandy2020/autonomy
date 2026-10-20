# 6. 传感器数据管线

### 6.1 消息类型

| 传感器 | 消息 | 典型频率 |
|--------|------|----------|
| 2D 激光 | `LaserScan` | 10–40 Hz |
| 3D / 深度 | `PointCloud2` | 10–20 Hz |
| RGB / 深度图 | `Image` | 15–30 Hz |
| IMU | `Imu` | 100–200 Hz |
| 里程计 | `Odometry` | 20–50 Hz |

定义：`commsgs/sensor_msgs.hpp`。

### 6.2 数据流

```
Driver/Sim → Sensor Buffer → TF 对齐
├─ ObstacleLayer (costmap)
├─ PerceptionServer（规划）
└─ Localization
```

### 6.3 驱动配置

```lua
laser = { topic = "/scan", frame_id = "laser_link", rate = 10.0 }
front_camera = {
    topic = "/camera/front/image_raw",
    frame_id = "camera_front_link",
    forward_targets = {"perception"},
}
```

### 6.4 ObservationBuffer

| 功能 | API / 参数 |
|------|------------|
| 入队 / 出队 | `bufferObservation` / `getObservations` |
| 过期 | `observation_keep_time` |

源码：`map/costmap_2d/observation_buffer.hpp`。

### 6.5 TF 与同步

```cpp
transform::Buffer::Instance()->lookupTransform(target, source, time, &tf);
// p_world = R * p_sensor + t
```

| 策略 | 场景 |
|------|------|
| 最近邻 | 低频融合 |
| 插值（SLERP） | 高频 IMU + 低频相机 |
| ApproximateTime | 多话题近似同步 |

### 6.6 激光管线（可用）

```
/scan → ObstacleLayer → TF → 射线清除/标记 → costmap
```

| 参数 | 含义 |
|------|------|
| `obstacle_range` | 标记最大距离 (m) |
| `raytrace_range` | 清除最大距离 (m) |
| `max_obstacle_height` | 高度上限 (m) |

### 6.7 相机 / 点云（规划）

```
Image → 解码 → 预处理 → ONNX → decode+NMS → Detection2DArray
PointCloud2 → TF → 体素 → 地面分割 → 聚类 → ObstacleArray / VoxelLayer
```

### 6.8 仿真对接

| 方式 | 传感器来源 |
|------|------------|
| Bridge / 静态层 | 注入或静态地图 |
| Gazebo | `/scan`、`/camera`、`/odom` |
| Stage（规划） | world 中激光 + 相机 |

### 6.9 参见

- [§3.1](03_math.md) · [§8](08_obstacle_perception.md) · [12 Simulation](../12_Simulation/00_guide.md)
