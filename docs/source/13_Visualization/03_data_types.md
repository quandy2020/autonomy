# 3. 数据类型

可视化数据来自 **commsgs**，语义对齐 ROS。

### 3.1 类型表

| 包 | 类型 | 用途 |
|----|------|------|
| sensor_msgs | LaserScan, PointCloud2, Image, Imu, Range | 传感器 |
| geometry_msgs | PoseStamped, TransformStamped, Point, Polygon | 几何 / TF |
| map_msgs | OccupancyGrid, OccupancyGridUpdate, GridMap | 地图 |
| planning_msgs / nav_msgs | Path, Odometry | 路径 / 里程计 |
| visualization_msgs | Marker, MarkerArray, InteractiveMarker | 标记 |
| vision_msgs | Detection2D/3DArray, BoundingBox* | 检测 |

定义：`commsgs/proto/visualization_msgs.proto` 等。

### 3.2 完整度

| 包 | struct / proto | ToProto/FromProto |
|----|----------------|-------------------|
| sensor / map / planning | — | |
| visualization_msgs | — | stub |

### 3.3 示例图

![路径可视化示例](./images/path.png)

![地图可视化示例](./images/map.png)

### 3.4 参见

- [14 Commsgs](../14_Commsgs/00_guide.md) · [§6 Foxglove](06_foxglove.md)
