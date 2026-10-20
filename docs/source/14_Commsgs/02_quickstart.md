# 2. 快速开始

写第一段 commsgs 构造或 Autolink Writer/Reader。

struct 与 proto 成对；大包体联调前实测。

### 构造 struct

```cpp
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"

geometry_msgs::PoseStamped pose;
pose.header.stamp = builtin_interfaces::Time::Now();
pose.header.frame_id = "map";
pose.pose.position.x = 1.0;
pose.pose.orientation.w = 1.0;

planning_msgs::Path path;
path.header = pose.header;
path.poses.push_back(pose);
```

### Writer / Reader

```cpp
auto writer = node->CreateWriter<commsgs::sensor_msgs::Imu>("/sensor/imu");
writer->Write(imu);

auto reader = node->CreateReader<commsgs::map_msgs::OccupancyGrid>(
    "/map", [](const auto& map) { /* ... */ });
```

### Service 边界

```cpp
auto path = commsgs::planning_msgs::FromProto(request->path());
response->set_is_valid(planner->IsPathValid(path));
```

### PointCloud2

```cpp
PointCloud2Modifier modifier(cloud);
modifier.setPointCloud2FieldsByString(3, "x", "y", "z");
modifier.resize(100);
```

### 头文件速查

| 需求 | 头文件 |
|------|--------|
| 时间 / Header | `builtin_interfaces.hpp` / `std_msgs.hpp` |
| 位姿 / TF | `geometry_msgs.hpp` |
| 传感器 / 地图 | `sensor_msgs.hpp` / `map_msgs.hpp` |
| 路径 | `planning_msgs.hpp` |
| Action | `proto/nav_msgs.pb.h` |

### 常见坑

| 现象 | 说明 |
|------|------|
| 边界全零 | [§9](09_survey.md) stub |
| Action 类型错 | 用 proto Action 名 |

### 延伸阅读

[§4 用法](04_usage.md) · [03 Communication](../03_Communication/00_guide.md)
