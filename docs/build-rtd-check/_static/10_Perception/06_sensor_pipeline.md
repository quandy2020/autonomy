# 6. 传感器数据管线

本文描述 Autonomy 感知子系统中传感器数据的采集、同步、坐标变换与分发机制。

---

## 6.1 传感器类型与消息

| 传感器 | 消息类型 | 命名空间 | 典型频率 |
|--------|----------|----------|----------|
| 2D 激光雷达 | `LaserScan` | `sensor_msgs` | 10–40 Hz |
| 3D 激光/深度相机 | `PointCloud2` | `sensor_msgs` | 10–20 Hz |
| RGB 相机 | `Image` | `sensor_msgs` | 15–30 Hz |
| 深度图 | `Image` (encoding=16UC1/32FC1) | `sensor_msgs` | 15–30 Hz |
| IMU | `Imu` | `sensor_msgs` | 100–200 Hz |
| 里程计 | `Odometry` | `planning_msgs` | 20–50 Hz |

消息定义位于 `autonomy/commsgs/sensor_msgs.hpp`。

---

## 6.2 数据流总览

```
┌─────────────┐     ┌──────────────┐     ┌─────────────────┐
│ Driver/Sim  │────→│ Sensor Buffer │────→│ Transform (TF)  │
│ (硬件/仿真)  │     │ (时间戳队列)   │     │ frame 对齐       │
└─────────────┘     └──────────────┘     └────────┬────────┘
                                                   │
                    ┌──────────────────────────────┼──────────────────┐
                    ▼                              ▼                  ▼
            ObstacleLayer                   PerceptionServer      Localization
            (costmap)                       (检测/聚类)            (定位)
```

---

## 6.3 驱动层配置

`config/driver/driver.lua` 定义传感器 topic 与转发目标：

```lua
-- 规划示例
laser = {
    topic = "/scan",
    frame_id = "laser_link",
    rate = 10.0,
}

front_camera = {
    topic = "/camera/front/image_raw",
    frame_id = "camera_front_link",
    forward_targets = {"perception"},  -- 预留
}
```

---

## 6.4 观测缓冲（ObservationBuffer）

`map/costmap_2d/observation_buffer.hpp` 实现传感器观测的时间缓冲：

| 功能 | 说明 |
|------|------|
| 入队 | `bufferObservation(obs)` 按时间戳入队 |
| 出队 | `getObservations(observations)` 获取有效窗口内观测 |
| 过期清理 | 超过 `observation_keep_time` 的观测自动丢弃 |

**用途**：costmap 更新时聚合多帧激光观测，提高鲁棒性。

---

## 6.5 坐标变换

所有感知算法需将传感器数据变换到统一参考系（通常为 `map` 或 `odom`）：

```cpp
// 查询 TF
geometry_msgs::TransformStamped tf;
transform::Buffer::Instance()->lookupTransform(
  target_frame, source_frame, time, &tf);

// 点变换
p_world = R * p_sensor + t;
```

**数学**：见 [03_math.md §3.1](03_math.md#31-坐标系与传感器模型)。

---

## 6.6 时间同步策略

| 策略 | 适用场景 | 说明 |
|------|----------|------|
| **最近邻** | 低频融合 | 取时间戳最接近的传感器帧 |
| **插值** | 高频 IMU + 低频相机 | 对位姿做 SLERP 插值 |
| **消息过滤器** | ROS 风格 | `message_filters::ApproximateTime` 近似同步 |

PerceptionServer 实现后，建议在 `@frequency` 循环中：

1. 收集各传感器最新帧（或同步帧组）
2. 统一变换到 `global_frame`
3. 分发给各 PerceptionInterface 插件

---

## 6.7 激光数据管线（当前可用）

```
LaserScan (/scan)
    │
    ▼
ObstacleLayer::laserScanCallback()
    │
    ├─ TF: laser_link → global_frame
    ├─ 射线追踪标记自由/障碍栅格
    └─ costmap 更新 → Planning / Control
```

**关键参数**（`costmap.lua`）：

| 参数 | 含义 |
|------|------|
| `obstacle_range` | 标记障碍的最大距离 (m) |
| `raytrace_range` | 射线清除自由空间的最大距离 (m) |
| `max_obstacle_height` | 障碍高度上限 (m) |

---

## 6.8 相机数据管线（规划）

```
Image (/camera/...)
    │
    ▼
PerceptionServer::OnImage()
    │
    ├─ 图像解码 (RGB/BGR)
    ├─ 预处理 (resize, normalize)  ← common/network
    ├─ ONNX 推理
    ├─ 后处理 (decode + NMS)
    └─ Detection2DArray 发布
```

---

## 6.9 点云数据管线（规划）

```
PointCloud2
    │
    ▼
PerceptionServer::OnPointCloud()
    │
    ├─ TF 变换到 map 系
    ├─ 体素下采样
    ├─ 地面分割 (RANSAC)
    ├─ 欧氏聚类
    └─ ObstacleArray / 写入 VoxelLayer
```

`VoxelLayer`（`map/costmap_2d/layers/voxel_layer.*`）支持 3D 点云体素化后投影到 2D costmap。

---

## 6.10 与仿真器的对接

| 仿真方式 | 传感器来源 | 配置 |
|----------|------------|------|
| `autonomy_nav_test` | 无真实传感器，静态地图 | costmap 静态层 |
| Gazebo（`autonomy_gazebo`） | `/scan`, `/camera`, `/odom` | `autonomy_ros` bridge |
| Stage（规划） | `cave.world` 中 hokuyolaser + camera | `simulation.lua` |

详见 [12 Simulation](../12_Simulation/index.rst)。
