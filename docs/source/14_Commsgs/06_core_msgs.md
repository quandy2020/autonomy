# 6. 核心消息类型

本文详述 `builtin_interfaces`、`std_msgs`、`geometry_msgs` 三个基础消息包。

## 6.1 builtin_interfaces

**头文件**：`autonomy/commsgs/builtin_interfaces.hpp`

时间与时长是全部 Stamped 消息的基础。

### 6.1.1 Time

| 字段 | 类型 | 说明 |
|------|------|------|
| `sec` | `int32` | Unix epoch 秒 |
| `nanosec` | `uint32` | 纳秒分量 [0, 10⁹) |

| 方法 | 说明 |
|------|------|
| `Time::Now()` | 当前系统时间 |
| `ToUnixTimeNanos()` | 合并为 64 位纳秒 |
| `Seconds()` | 浮点秒 |
| `Nanoseconds()` | 总纳秒（32 位，注意溢出） |
| `Min()` / `Max()` | 边界值 |

```cpp
auto t = commsgs::builtin_interfaces::Time::Now();
auto t2 = commsgs::builtin_interfaces::Time(100, 500000000);  // 100.5s
```

### 6.1.2 Duration

| 字段 | 类型 | 说明 |
|------|------|------|
| `duration_` | `int64_t` | 内部纳秒存储 |

支持与 `std::chrono::duration` 互转，提供 `+` `-` `*` 运算符（带溢出检查）。

```cpp
auto d = commsgs::builtin_interfaces::Duration::FromSeconds(1.5);
auto elapsed = commsgs::builtin_interfaces::Time::Now() - start_time;
```

## 6.2 std_msgs

**头文件**：`autonomy/commsgs/std_msgs.hpp`

### 6.2.1 Header

| 字段 | 类型 | 说明 |
|------|------|------|
| `stamp` | `builtin_interfaces::Time` | 时间戳 |
| `frame_id` | `std::string` | 坐标系名称 |

所有 `*Stamped` 消息的公共前缀。

### 6.2.2 其他类型

| 类型 | 字段 | 用途 |
|------|------|------|
| `ColorRGBA` | `r, g, b, a` (float) | 可视化颜色 |
| `String` | `data` (string) | 字符串包装 |
| `Float32MultiArray` | `layout`, `data` | 多维浮点数组 |
| `MultiArrayLayout` | `dim`, `data_offset` | 数组布局描述 |
| `MultiArrayDimension` | `label`, `size`, `stride` | 单维描述 |

> `MultiArray*` 系列自 Foxy 起标记为 deprecated，建议使用语义化消息替代。

### 6.2.3 转换完整度

| 类型 | ToProto | FromProto |
|------|---------|-----------|
| `Header` | ✅ | ✅ |
| `ColorRGBA` | ✅ | ✅ |
| `String` | ✅ | ✅ |
| `MultiArray*` | ✅ | ✅ |

## 6.3 geometry_msgs

**头文件**：`autonomy/commsgs/geometry_msgs.hpp`

导航栈中使用最频繁的消息包。

### 6.3.1 基础向量与点

| 类型 | 字段 | 语义 |
|------|------|------|
| `Vector3` | `x, y, z` (float) | 自由向量（锚定原点，变换时仅旋转） |
| `Point` | `x, y, z` (float) | 空间点（变换时平移+旋转） |
| `Point32` | `x, y, z` (float) | 32 位精度点 |
| `Quaternion` | `x, y, z, w` (float) | 四元数旋转 |
| `Polygon` | `vector<Point32>` | 多边形顶点 |

### 6.3.2 位姿与运动

| 类型 | 组成 | 用途 |
|------|------|------|
| `Pose` | `Point position` + `Quaternion orientation` | 6-DOF 位姿 |
| `Pose2D` | `x, y, theta` (double) | 平面位姿 |
| `Twist` | `Vector3 linear` + `Vector3 angular` | 6-DOF 速度 |
| `Twist2D` | `x, y, theta` (double) | 平面速度 |
| `Accel` | `Vector3 linear` + `Vector3 angular` | 加速度 |
| `Wrench` | `Vector3 force` + `Vector3 torque` | 力/力矩 |

### 6.3.3 Stamped 变体

| 类型 | 额外字段 | 典型 topic |
|------|----------|-----------|
| `PoseStamped` | `header` + `pose` | `/goal_pose`, 路径点 |
| `Pose2DStamped` | `header` + `pose` | 平面目标 |
| `TwistStamped` | `header` + `twist` | `/cmd_vel` |
| `Twist2DStamped` | `header` + `velocity` | 平面速度命令 |
| `PointStamped` | `header` + `point` | 标注点 |
| `QuaternionStamped` | `header` + `quaternion` | 方向 |
| `Vector3Stamped` | `header` + `vector` | 力/加速度向量 |

### 6.3.4 协方差变体

| 类型 | 协方差大小 | 顺序 |
|------|-----------|------|
| `PoseWithCovariance` | 36 (6×6) | x, y, z, roll, pitch, yaw |
| `TwistWithCovariance` | 36 (6×6) | 同上 |
| `AccelWithCovariance` | 36 (6×6) | 同上 |

对应 `*Stamped` 变体：`PoseWithCovarianceStamped`、`TwistWithCovarianceStamped` 等。

### 6.3.5 变换

| 类型 | 字段 | 说明 |
|------|------|------|
| `Transform` | `Vector3 translation` + `Quaternion rotation` | 刚体变换 |
| `TransformStamped` | `header` + `child_frame_id` + `transform` | TF 边 |
| `TransformStampeds` | `vector<TransformStamped>` | 批量变换 |
| `VelocityStamped` | `header` + `body_frame_id` + `Twist velocity` | 体坐标系速度 |

**TF 约定**：

```
header.frame_id     = 父坐标系（reference frame）
child_frame_id      = 子坐标系（target frame）
transform           = 从 child 到 parent 的变换
```

### 6.3.6 集合类型

| 类型 | 组成 | 用途 |
|------|------|------|
| `PoseArray` | `header` + `vector<Pose>` | 粒子滤波器粒子 |
| `Pose2DArray` | `header` + `vector<Pose2D>` | 平面粒子 |

### 6.3.7 Apollo 扩展

| 类型 | 字段 | 用途 |
|------|------|------|
| `PointENU` | `x, y, z` (double) | UTM/ENU 坐标 |
| `PointLLH` | `lon, lat, height` (double) | 经纬高（WGS84） |
| `Inertia` | `m, com, ixx..izz` | 刚体惯性张量 |
| `InertiaStamped` | `header` + `inertia` | 带时间戳惯性 |

### 6.3.8 转换完整度

| 类别 | 完整度 | 备注 |
|------|--------|------|
| Vector3 / Point / Quaternion | ✅ 完整 | |
| Pose / Twist / Transform | ✅ 完整 | |
| *Stamped 系列 | ✅ 大部分完整 | |
| *WithCovariance | ✅ 完整 | |
| Polygon | ⏳ 部分 | `ToProto` 可能遗漏 points |
| PoseArray | ⏳ 部分 | header 转换待补全 |
| PointENU / PointLLH | ✅ 完整 | |

## 6.4 使用频率排序

导航栈中最常用的 geometry_msgs 类型：

| 优先级 | 类型 | 使用模块 |
|--------|------|----------|
| ★★★ | `PoseStamped` | planning, control, navigator |
| ★★★ | `Twist` / `TwistStamped` | control |
| ★★★ | `TransformStamped` | transform, localization |
| ★★☆ | `Pose` / `Pose2D` | checker, 几何计算 |
| ★★☆ | `PoseWithCovariance` | localization, odom |
| ★☆☆ | `Polygon` | costmap 禁区 |
| ★☆☆ | `PointENU` / `PointLLH` | 高精地图 |

## 6.5 代码示例：完整位姿链路

```cpp
// 1. 里程计位姿（带协方差）
commsgs::geometry_msgs::PoseWithCovarianceStamped loc;
loc.header.stamp = commsgs::builtin_interfaces::Time::Now();
loc.header.frame_id = "map";
loc.pose.pose.position.x = 1.0;
loc.pose.covariance.resize(36, 0.0);
loc.pose.covariance[0] = 0.01;   // x 方差
loc.pose.covariance[7] = 0.01;   // y 方差
loc.pose.covariance[35] = 0.05;  // yaw 方差

// 2. 提取 PoseStamped 用于规划
commsgs::geometry_msgs::PoseStamped current;
current.header = loc.header;
current.pose = loc.pose.pose;

// 3. 计算速度命令
commsgs::geometry_msgs::TwistStamped cmd;
cmd.header.stamp = commsgs::builtin_interfaces::Time::Now();
cmd.header.frame_id = "base_link";
cmd.twist.linear.x = 0.5;
cmd.twist.angular.z = 0.1;
```
