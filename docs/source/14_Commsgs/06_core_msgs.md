# 6. 核心消息类型

### 6.1 builtin_interfaces

头文件：`builtin_interfaces.hpp`。

| 类型 | 字段 / API |
|------|------------|
| `Time` | `sec`,`nanosec`；`Now()`、`ToUnixTimeNanos()`、比较 |
| `Duration` | `int64` 纳秒；与 chrono 互转；`FromSeconds` |

### 6.2 std_msgs

| 类型 | 字段 | 转换 |
|------|------|------|
| `Header` | `stamp`,`frame_id` | — |
| `ColorRGBA` | `r,g,b,a` | — |
| `String` | `data` | — |
| `Float32MultiArray` 等 | layout + data | （deprecated 倾向） |

### 6.3 geometry_msgs

| 类别 | 类型 |
|------|------|
| 点/向量 | `Vector3`,`Point`,`Point32`,`Quaternion`,`Polygon` |
| 位姿/运动 | `Pose`,`Pose2D`,`Twist`,`Twist2D`,`Accel`,`Wrench` |
| Stamped | `PoseStamped`,`TwistStamped`,`PointStamped`,… |
| 协方差 | `Pose/Twist/AccelWithCovariance`（36=6×6） |
| TF | `Transform`,`TransformStamped`（父=`frame_id`，子=`child_frame_id`） |
| 集合 | `PoseArray`,`Pose2DArray` |
| Apollo 扩展 | `PointENU`,`PointLLH`,`Inertia` |

| 转换 | 状态 |
|------|------|
| 基础 / Stamped / Covariance | — |
| Polygon / PoseArray | 部分 |

### 6.4 常用优先级

| 类型 | 模块 |
|------|------|
| PoseStamped, Twist, TransformStamped | planning/control/transform |
| PoseWithCovariance | localization |
| Polygon, PointENU/LLH | costmap / 高精地图（较少） |

```cpp
PoseWithCovarianceStamped loc;
loc.header.frame_id = "map";
loc.pose.covariance.resize(36, 0.0);
TwistStamped cmd; cmd.twist.linear.x = 0.5;
```

### 6.5 参见
