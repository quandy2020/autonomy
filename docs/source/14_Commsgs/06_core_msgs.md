# 6. 核心消息类型

写规划/控制/TF 代码时查 Header、位姿、协方差字段约定。

builtin / std / geometry 是栈内**最稳**的一层；联调优先用这些类型。

| 需求 | 类型 |
|------|------|
| 单点目标 | PoseStamped |
| 速度指令 | Twist / TwistStamped |
| 坐标变换 | TransformStamped |
| 带不确定度位姿 | PoseWithCovariance* |

### builtin_interfaces

头文件：`builtin_interfaces.hpp`。

| 类型 | 说明 |
|------|------|
| `Time` | `sec`, `nanosec`；`Now()`、`ToUnixTimeNanos()` |
| `Duration` | 内部纳秒；与 `std::chrono` 互转 |

### std_msgs

| 类型 | 字段 |
|------|------|
| `Header` | `stamp`, `frame_id` |
| `ColorRGBA` | `r,g,b,a` |
| `String` | `data` |

### geometry_msgs（高频）

| 类别 | 类型 |
|------|------|
| 基础 | Vector3, Point, Quaternion, Polygon |
| 位姿/运动 | Pose, Twist, Accel |
| Stamped | PoseStamped, TwistStamped, … |
| 协方差 | *WithCovariance（36 元 = 6×6 行主序） |
| TF | TransformStamped（父=`header.frame_id`，子=`child_frame_id`） |
| 扩展 | PointENU, PointLLH, Inertia |

**转换：** 基础与 Stamped 较完整；`Polygon`、`PoseArray` 等部分路径需实测（[§9](09_survey.md)）。

### 模块常用组合

| 类型 | 典型模块 |
|------|----------|
| PoseStamped, Twist, TransformStamped | planning / control / transform |
| PoseWithCovarianceStamped | localization |
| Polygon | costmap 区域（较少） |

```cpp
PoseWithCovarianceStamped loc;
loc.header.frame_id = "map";
loc.pose.covariance.resize(36, 0.0);
TwistStamped cmd;
cmd.twist.linear.x = 0.5;
```

### 常见坑

| 现象 | 说明 |
|------|------|
| TF 父子反了 | child 在 `child_frame_id` |
| 协方差全 0 被当无效 | 某些模块用 -1 表示未知（Imu 等另论） |

### 延伸阅读

[§7](07_sensor_map_msgs.md) · [§8](08_nav_planning_msgs.md)
