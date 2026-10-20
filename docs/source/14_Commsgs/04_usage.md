(commsgs-usage)=
# 4. 使用指南

### 4.1 Include

```cpp
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
// 边界：proto/nav_msgs.pb.h, proto/error_code.pb.h
```

### 4.2 Header 与 frame

```cpp
header.stamp = Time::Now();
header.frame_id = "map"; // 或 odom / base_link / laser
```

### 4.3 常用构造

```cpp
// Pose / Twist / TF
pose.position = {1,2,0}; pose.orientation.w = 1;
cmd_vel.linear.x = 0.5; cmd_vel.angular.z = 0.2;
tf.header.frame_id = "map"; tf.child_frame_id = "base_link";

// Path / Odometry / OccupancyGrid
path.header.frame_id = "map"; path.poses.push_back(ps);
odom.header.frame_id = "odom"; odom.child_frame_id = "base_link";
grid.info.resolution = 0.05; grid.data.assign(w*h, 0); // -1未知,100占据
```

### 4.4 扩展消息

1. `proto/my_msgs.proto` → 2. `my_msgs.hpp` struct → 3. `my_msgs.cpp` ToProto/FromProto → 重编译。

### 4.5 集成模式

| 场景 | 类型 | 转换 |
|------|------|------|
| 算法内部 / 同进程 | struct | 无 |
| Writer/Reader | struct | 框架内 |
| Service/Action | proto | 入口 FromProto，出口 ToProto |

### 4.6 排错与往返测试

| 现象 | 处理 |
|------|------|
| 无数据 / 字段全零 | 类型匹配；查 ToProto stub |
| TF 错 | frame / child_frame 勿反 |
| 协方差无效 | 长度 36（6×6） |
| PointCloud 越界 | 先 `Modifier::resize` |

```cpp
auto restored = FromProto(ToProto(original));
EXPECT_EQ(restored.header.frame_id, "test");
```

### 4.7 参见
