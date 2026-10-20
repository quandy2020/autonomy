(commsgs-usage)=
# 4. 使用指南

日常写消息、扩包或排查 ToProto 问题。

struct 与 proto 成对；大包体联调前实测。

### Include 与 frame

```cpp
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
```

`header.stamp = Time::Now()`；`frame_id` 常用 `map` / `odom` / `base_link` / `laser_link`。

### 常用构造

Path、Odometry、OccupancyGrid（`data`：-1 未知，0 自由，100 占据）、Twist、TransformStamped（勿反 parent/child）。

### 扩展新消息

1. `proto/my_msgs.proto`  
2. `my_msgs.hpp`  
3. `my_msgs.cpp` ToProto/FromProto  
4. 往返单测

### 集成模式

| 场景 | 类型 | 转换 |
|------|------|------|
| 同进程算法 | struct | 无 |
| Pub/Sub | struct | 框架 |
| Service/Action | proto | 边界 FromProto/ToProto |

### 排错

| 现象 | 处理 |
|------|------|
| 字段全零 | stub ToProto |
| TF 错 | child_frame_id |
| 协方差无效 | 长度 36 |
| 点云越界 | 先 resize |

```cpp
auto restored = FromProto(ToProto(original));
```

### 延伸阅读

[§3 Schema](03_schema.md) · [§9 综述](09_survey.md)
