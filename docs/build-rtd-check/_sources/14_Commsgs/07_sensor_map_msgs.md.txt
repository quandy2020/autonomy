# 7. 传感器与地图消息

本文详述 `sensor_msgs`、`map_msgs` 及 PointCloud2 工具链。

## 7.1 sensor_msgs

**头文件**：`autonomy/commsgs/sensor_msgs.hpp`

### 7.1.1 激光雷达

#### LaserScan

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs::Header` | 时间戳 + `frame_id`（通常为 `laser`） |
| `angle_min` | `float` | 起始角度 (rad) |
| `angle_max` | `float` | 结束角度 (rad) |
| `angle_increment` | `float` | 角度步进 (rad) |
| `time_increment` | `float` | 测量间隔 (s) |
| `scan_time` | `float` | 扫描周期 (s) |
| `range_min` / `range_max` | `float` | 有效量程 (m) |
| `ranges` | `vector<float>` | 距离读数 |
| `intensities` | `vector<float>` | 反射强度 |

```cpp
commsgs::sensor_msgs::LaserScan scan;
scan.header.frame_id = "laser";
scan.angle_min = -M_PI;
scan.angle_max = M_PI;
scan.angle_increment = M_PI / 180.0;
scan.range_min = 0.1f;
scan.range_max = 30.0f;
scan.ranges.resize(360, std::numeric_limits<float>::infinity());
```

### 7.1.2 IMU

#### Imu

| 字段组 | 字段 | 说明 |
|--------|------|------|
| 方向 | `orientation` (Quaternion) | 融合后姿态 |
| 角速度 | `angular_velocity` (Vector3) | rad/s |
| 线加速度 | `linear_acceleration` (Vector3) | m/s² |
| 协方差 | `orientation_covariance[9]` | 3×3 行主序 |
| 协方差 | `angular_velocity_covariance[9]` | 3×3 |
| 协方差 | `linear_acceleration_covariance[9]` | 3×3 |

- 协方差 `-1` 表示未知
- `orientation` 全零表示不提供方向估计

### 7.1.3 点云

#### PointCloud2

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs::Header` | 时间戳 + 坐标系 |
| `height` | `uint32` | 行数（无序点云为 1） |
| `width` | `uint32` | 列数（总点数 = height × width） |
| `fields` | `vector<PointField>` | 字段描述 |
| `is_bigendian` | `bool` | 字节序 |
| `point_step` | `uint32` | 单点字节数 |
| `row_step` | `uint32` | 单行字节数 |
| `data` | `vector<uint8>` | 二进制点云数据 |
| `is_dense` | `bool` | 是否无 NaN/Inf |

#### PointField

| 字段 | 类型 | 说明 |
|------|------|------|
| `name` | `string` | 字段名（`"x"`, `"y"`, `"z"`, `"rgb"`, `"intensity"`） |
| `offset` | `uint32` | 在点内的字节偏移 |
| `datatype` | `uint8` | 数据类型枚举 |
| `count` | `uint32` | 元素数量 |

**datatype 枚举**：

| 值 | 常量 | C++ 类型 |
|----|------|----------|
| 1 | `INT8` | `int8_t` |
| 2 | `UINT8` | `uint8_t` |
| 3 | `INT16` | `int16_t` |
| 4 | `UINT16` | `uint16_t` |
| 5 | `INT32` | `int32_t` |
| 6 | `UINT32` | `uint32_t` |
| 7 | `FLOAT32` | `float` |
| 8 | `FLOAT64` | `double` |

> `PointCloud`（旧格式，含 `ChannelFloat32`）已标记 deprecated，推荐使用 `PointCloud2`。

### 7.1.4 相机

| 类型 | 关键字段 | 用途 |
|------|----------|------|
| `Image` | `height, width, encoding, step, data` | 原始图像 |
| `CompressedImage` | `format, data` | 压缩图像（JPEG/PNG） |
| `CameraInfo` | `k[9], d, r[9], p[12], distortion_model` | 相机标定 |
| `RegionOfInterest` | `x_offset, y_offset, height, width` | ROI |

**CameraInfo 矩阵**：

```
K = [fx  0 cx]     内参（3×3 行主序，9 元素）
    [ 0 fy cy]
    [ 0  0  1]

P = [fx'  0  cx' Tx]   投影矩阵（3×4 行主序，12 元素）
    [ 0  fy' cy' Ty]
    [ 0   0   1   0]
```

### 7.1.5 其他传感器

| 类型 | 用途 |
|------|------|
| `Range` | 超声波/红外测距 |
| `Joy` | 手柄输入 |
| `Illuminance` | 光照度 |
| `BatteryState` | 电池状态（proto 别名） |

### 7.1.6 转换完整度

| 类型 | ToProto | FromProto |
|------|---------|-----------|
| `LaserScan` | ⏳ 部分 | ⏳ 部分 |
| `Imu` | ⏳ 部分 | ⏳ 部分 |
| `PointCloud2` | ⏳ 部分 | ⏳ 部分 |
| `Image` | ⏳ 部分 | ⏳ 部分 |
| `CameraInfo` | ⏳ 部分 | ⏳ 部分 |

## 7.2 PointCloud2 工具链

### 7.2.1 PointCloud2Modifier

**头文件**：`autonomy/commsgs/point_cloud2_iterator.hpp`

便捷设置字段与大小：

```cpp
commsgs::sensor_msgs::PointCloud2Modifier modifier(cloud);
modifier.setPointCloud2FieldsByString(4, "x", "y", "z", "intensity");
modifier.resize(num_points);
```

| 方法 | 说明 |
|------|------|
| `setPointCloud2Fields(n, ...)` | 按名称/类型/数量设置字段 |
| `setPointCloud2FieldsByString(n, ...)` | 按字符串简写（`"xyz"`, `"rgb"`） |
| `resize(n)` | 设置点数并分配 data |
| `reserve(n)` | 预分配 |
| `clear()` | 清空 |

### 7.2.2 PointCloud2Iterator

按字段名遍历点云：

```cpp
commsgs::sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
commsgs::sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
commsgs::sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

for (size_t i = 0; i < cloud.width; ++i, ++iter_x, ++iter_y, ++iter_z) {
    *iter_x = points[i].x;
    *iter_y = points[i].y;
    *iter_z = points[i].z;
}
```

只读版本：`PointCloud2ConstIterator<T>`。

**使用方**：`autonomy/map/costmap_2d/layers/obstacle_layer.cpp`、`voxel_layer.cpp`。

### 7.2.3 point_field_conversion

**头文件**：`autonomy/commsgs/point_field_conversion.hpp`

编译期类型映射：

```cpp
// enum → C++ 类型
using T = commsgs::sensor_msgs::pointFieldTypeAsType<
    commsgs::sensor_msgs::PointField::FLOAT32>::type;  // float

// C++ 类型 → enum
constexpr auto dt = commsgs::sensor_msgs::typeAsPointFieldType<float>::value;

// 从 buffer 读取
float val = commsgs::sensor_msgs::readPointCloud2BufferValue<float>(data_ptr, datatype);
```

## 7.3 map_msgs

**头文件**：`autonomy/commsgs/map_msgs.hpp`

### 7.3.1 静态栅格地图

#### OccupancyGrid

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs::Header` | `frame_id` 通常为 `"map"` |
| `info` | `MapMetaData` | 分辨率、尺寸、原点 |
| `data` | `vector<int16>` | 行主序栅格值 |

**data 值语义**：

| 值 | 含义 |
|----|------|
| `-1` | 未知 |
| `0` | 自由 |
| `1–99` | 概率占据 |
| `100` | 完全占据 |

#### MapMetaData

| 字段 | 类型 | 说明 |
|------|------|------|
| `map_load_time` | `builtin_interfaces::Time` | 地图加载时间 |
| `resolution` | `float` | m/cell |
| `width` / `height` | `uint32` | 栅格尺寸 (cells) |
| `origin` | `geometry_msgs::Pose` | 左下角 (0,0) 的世界坐标 |

#### OccupancyGridUpdate

增量更新：`x, y, width, height` 定义矩形区域 + `vector<int8> data`。

### 7.3.2 代价地图（Nav2 风格）

| 类型 | 说明 |
|------|------|
| `Costmap` | 完整代价地图（`header` + `CostmapMetaData` + `data`） |
| `CostmapUpdate` | 增量更新 |
| `CostmapMetaData` | 分辨率、尺寸、原点、layer 名 |
| `CostmapFilterInfo` | 速度限制过滤器元数据 |
| `GetCostmapRequest/Response` | 获取代价地图服务 |
| `LoadMapRequest/Response` | 加载地图服务 |
| `SaveMapRequest/Response` | 保存地图服务 |

**Costmap data 值语义**：

| 值 | 含义 |
|----|------|
| `0` | 自由 |
| `1–252` | 代价递增 |
| `253` | 内切障碍 |
| `254` | 致命障碍 |
| `255` | 未知 |

### 7.3.3 GridMap（2.5D 多层栅格）

| 类型 | 说明 |
|------|------|
| `GridMapInfo` | 分辨率、尺寸、中心位姿 |
| `GridMap` | `layers`（层名）+ `data`（行主序 float 数组，按层排列） |
| `basic_layers` | 决定有效性的基础层 |

典型层名：`"elevation"`, `"traversability"`, `"cost"`。

### 7.3.4 3D 地图

| 类型 | 说明 |
|------|------|
| `Octomap` | 八叉树二进制序列化 |
| `OctomapWithPose` | 带位姿的八叉树 |
| `VoxelGrid` | 体素栅格 |

### 7.3.5 辅助类型

| 类型 | 说明 |
|------|------|
| `GridCells` | 2D 栅格单元点集 |

### 7.3.6 转换完整度

| 类型 | ToProto | FromProto |
|------|---------|-----------|
| `OccupancyGrid` | ⏳ 部分 | ⏳ 部分 |
| `Costmap` | ⏳ 部分 | ⏳ 部分 |
| `GridMap` | ⏳ 部分 | ⏳ 部分 |
| `MapMetaData` | ✅ 大部分 | ✅ 大部分 |

## 7.4 数据流：传感器 → 代价地图

```
LaserScan / PointCloud2
        │
        ▼
  obstacle_layer（PointCloud2Iterator 解析 xyz）
        │
        ▼
  Costmap2D（内部 uint8 数组）
        │
        ▼
  map_msgs::Costmap（发布/可视化）
```

## 7.5 stereo_msgs

**头文件**：`autonomy/commsgs/stereo_msgs.hpp`

| 类型 | 说明 |
|------|------|
| `DisparityImage` | `sensor_msgs::Image` + 视差范围/增量/基线等参数 |

用于双目立体视觉深度估计。

## 7.6 使用建议

| 场景 | 推荐消息 | 注意事项 |
|------|----------|----------|
| 2D 导航地图 | `OccupancyGrid` | `data` 行主序，origin 为左下角 |
| 实时避障 | `Costmap` / `CostmapUpdate` | 253/254 为障碍 |
| 3D 地形 | `GridMap` | 多层 float，注意 `basic_layers` |
| 激光输入 | `LaserScan` | 检查 `ranges.size()` 与角度一致性 |
| 3D 障碍 | `PointCloud2` | 使用 Modifier + Iterator 构造 |
| 相机标定 | `CameraInfo` | K 全零表示未标定 |
