# 2. 快速开始

### 2.1 三步使用

1. 在 C++ 源文件中 `#include` 对应消息头文件
2. 构造 C++ struct，填充字段
3. 通过 Autolink 发布/订阅时，在边界调用 `ToProto` / `FromProto`

> commsgs 无独立 CMake 目标，随 `libautonomy` 一并编译安装；无需额外配置。

### 2.2 构造并填充消息

```cpp
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"

using namespace autonomy::commsgs;

// 构造 PoseStamped
geometry_msgs::PoseStamped pose;
pose.header.stamp = builtin_interfaces::Time::Now();
pose.header.frame_id = "map";
pose.pose.position.x = 1.0;
pose.pose.position.y = 2.0;
pose.pose.orientation.w = 1.0;

// 构造 Path
planning_msgs::Path path;
path.header = pose.header;
path.poses.push_back(pose);
```

### 2.3 Autolink 发布（Writer）

```cpp
#include "autonomy/commsgs/sensor_msgs.hpp"

auto writer = node->CreateWriter<commsgs::sensor_msgs::Imu>("/sensor/imu");

commsgs::sensor_msgs::Imu imu;
imu.header.stamp = commsgs::builtin_interfaces::Time::Now();
imu.header.frame_id = "imu_link";
imu.angular_velocity.z = 0.1;
imu.linear_acceleration.x = 9.8;

writer->Write(imu);  // 框架内部处理序列化
```

### 2.4 Autolink 订阅（Reader）

```cpp
auto callback = [](const std::shared_ptr<commsgs::map_msgs::OccupancyGrid>& map) {
    const auto& info = map->info;
    // 使用 map->data 访问栅格数据
};

auto reader = node->CreateReader<commsgs::map_msgs::OccupancyGrid>("/map", callback);
```

### 2.5 Service 边界转换

`PlannerServer` 在 Autolink 服务边界使用 proto 类型，内部转换为 C++ struct：

```cpp
#include "autonomy/commsgs/planning_msgs.hpp"

namespace nav_proto = commsgs::proto::nav_msgs;

// 服务回调：request 为 proto，内部转 C++
const auto path = commsgs::planning_msgs::FromProto(request->path());
bool valid = planner->IsPathValid(path);
response->set_is_valid(valid);
```

### 2.6 手动 Proto 序列化

```cpp
#include "autonomy/commsgs/geometry_msgs.hpp"

commsgs::geometry_msgs::PoseStamped pose;
// ... 填充 pose

// C++ → Proto → 字节流
auto proto = commsgs::geometry_msgs::ToProto(pose);
std::string bytes;
proto.SerializeToString(&bytes);

// 字节流 → Proto → C++
commsgs::proto::geometry_msgs::PoseStamped proto_in;
proto_in.ParseFromString(bytes);
auto pose_out = commsgs::geometry_msgs::FromProto(proto_in);
```

### 2.7 PointCloud2 快速示例

```cpp
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/commsgs/point_cloud2_iterator.hpp"

commsgs::sensor_msgs::PointCloud2 cloud;
commsgs::sensor_msgs::PointCloud2Modifier modifier(cloud);
modifier.setPointCloud2FieldsByString(3, "x", "y", "z");
modifier.resize(100);

commsgs::sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
commsgs::sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
commsgs::sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

for (size_t i = 0; i < 100; ++i, ++iter_x, ++iter_y, ++iter_z) {
    *iter_x = static_cast<float>(i);
    *iter_y = 0.0f;
    *iter_z = 0.0f;
}
```

### 2.8 头文件速查

| 需求 | 头文件 |
|------|--------|
| 时间戳 | `autonomy/commsgs/builtin_interfaces.hpp` |
| Header / 颜色 | `autonomy/commsgs/std_msgs.hpp` |
| 位姿 / 速度 / TF | `autonomy/commsgs/geometry_msgs.hpp` |
| 传感器数据 | `autonomy/commsgs/sensor_msgs.hpp` |
| 地图 / 代价地图 | `autonomy/commsgs/map_msgs.hpp` |
| 路径 / 里程计 | `autonomy/commsgs/planning_msgs.hpp` |
| Nav2 Action proto | `autonomy/commsgs/proto/nav_msgs.pb.h` |
| 错误码 | `autonomy/commsgs/proto/error_code.pb.h` |
| PointCloud2 工具 | `autonomy/commsgs/point_cloud2_iterator.hpp` |

### 2.9 下一步

- 理解双层设计：[§3 消息模型](03_schema.md)
- 深入架构：[§5 模块架构](05_architecture.md)
- 按包查阅类型定义：[§6–§8](06_core_msgs.md)
