# 7. 传感器与地图消息

### 7.1 sensor_msgs

| 类型 | 关键字段 |
|------|----------|
| `LaserScan` | angle_*、range_*、`ranges`/`intensities` |
| `Imu` | orientation、angular_velocity、linear_acceleration + 3×3 协方差（-1=未知） |
| `PointCloud2` | height/width、fields、point_step、data、is_dense |
| `PointField` | name、offset、datatype(1–8)、count |
| `Image` / `CompressedImage` | height/width/encoding；format/data |
| `CameraInfo` | K[9]、D、R[9]、P[12] |
| 其他 | Range、Joy、Illuminance、BatteryState |

旧 `PointCloud` deprecated。ToProto： 部分。

### 7.2 PointCloud2 工具

| API | 作用 |
|-----|------|
| `PointCloud2Modifier` | `setPointCloud2FieldsByString`、`resize` |
| `PointCloud2Iterator<T>` | 按字段遍历 |
| `point_field_conversion` | enum ↔ C++ 类型、buffer 读 |

使用方：`obstacle_layer`、`voxel_layer`。

### 7.3 map_msgs

| 类型 | 说明 |
|------|------|
| `OccupancyGrid` | info + data：-1 未知，0 自由，100 占据 |
| `MapMetaData` | resolution、width/height、origin |
| `OccupancyGridUpdate` | 矩形增量 |
| `Costmap` / Update / MetaData | Nav2 风格；0 自由，253 内切，254 致命，255 未知 |
| `GridMap` | 多层 float（elevation 等） |
| `Octomap` / `VoxelGrid` / `GridCells` | 3D / 辅助 |

转换：OccupancyGrid/Costmap/GridMap ；MapMetaData 大部分 。

### 7.4 数据流与建议

```
LaserScan/PointCloud2 → obstacle_layer → Costmap2D → map_msgs::Costmap
```

| 场景 | 消息 |
|------|------|
| 2D 地图 / 避障 | OccupancyGrid / Costmap |
| 激光 / 3D 障碍 | LaserScan / PointCloud2+Iterator |
| 地形 | GridMap |

`stereo_msgs::DisparityImage`：视差图参数。

### 7.5 参见

- [§6](06_core_msgs.md) · [10 Perception · 障碍](../10_Perception/08_obstacle_perception.md)
