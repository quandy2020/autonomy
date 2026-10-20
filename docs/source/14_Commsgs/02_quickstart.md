# 2. 快速开始

随 `libautonomy` 编译；无独立 CMake 目标。

### 2.1 构造

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

### 2.2 Writer / Reader

```cpp
auto writer = node->CreateWriter<commsgs::sensor_msgs::Imu>("/sensor/imu");
writer->Write(imu);

auto reader = node->CreateReader<commsgs::map_msgs::OccupancyGrid>(
    "/map", [](const auto& map) { /* map->info, map->data */ });
```

### 2.3 Service 边界与手动序列化

```cpp
auto path = commsgs::planning_msgs::FromProto(request->path());
response->set_is_valid(planner->IsPathValid(path));

auto proto = commsgs::geometry_msgs::ToProto(pose);
proto.SerializeToString(&bytes);
```

### 2.4 PointCloud2

```cpp
PointCloud2Modifier modifier(cloud);
modifier.setPointCloud2FieldsByString(3, "x", "y", "z");
modifier.resize(100);
PointCloud2Iterator<float> iter_x(cloud, "x");
// ...
```

### 2.5 头文件速查

| 需求 | 头文件 |
|------|--------|
| 时间 / Header | `builtin_interfaces.hpp` / `std_msgs.hpp` |
| 位姿 / TF | `geometry_msgs.hpp` |
| 传感器 / 地图 | `sensor_msgs.hpp` / `map_msgs.hpp` |
| 路径 | `planning_msgs.hpp` |
| Action / 错误码 | `proto/nav_msgs.pb.h` / `proto/error_code.pb.h` |
| 点云工具 | `point_cloud2_iterator.hpp` |

### 2.6 参见
