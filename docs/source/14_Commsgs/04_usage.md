(commsgs-usage)=
# 4. 使用指南

### 4.1 包含头文件

按消息包粒度包含，避免全量引入：

```cpp
#include "autonomy/commsgs/geometry_msgs.hpp"    // 位姿、速度
#include "autonomy/commsgs/planning_msgs.hpp"    // Path、Odometry
#include "autonomy/commsgs/map_msgs.hpp"         // OccupancyGrid、Costmap
#include "autonomy/commsgs/sensor_msgs.hpp"      // Imu、LaserScan、PointCloud2
```

Proto 生成头文件仅在 Autolink 服务/Action 边界需要：

```cpp
#include "autonomy/commsgs/proto/nav_msgs.pb.h"
#include "autonomy/commsgs/proto/error_code.pb.h"
```

### 4.2 构造 Header

所有 Stamped 消息的基础：

```cpp
commsgs::std_msgs::Header header;
header.stamp = commsgs::builtin_interfaces::Time::Now();
header.frame_id = "base_link";
```

**frame_id 约定**（与 ROS REP 一致）：

| frame_id | 典型场景 |
|----------|----------|
| `"map"` | 全局地图坐标系 |
| `"odom"` | 里程计坐标系 |
| `"base_link"` | 机器人本体 |
| `"laser"` / `"camera_optical_frame"` | 传感器外参 |

### 4.3 构造几何消息

#### Pose / PoseStamped

```cpp
commsgs::geometry_msgs::Pose pose;
pose.position.x = 1.0;
pose.position.y = 2.0;
pose.position.z = 0.0;
pose.orientation.w = 1.0;  // 无旋转时 w=1

commsgs::geometry_msgs::PoseStamped pose_stamped;
pose_stamped.header.stamp = commsgs::builtin_interfaces::Time::Now();
pose_stamped.header.frame_id = "map";
pose_stamped.pose = pose;
```

#### Twist（速度命令）

```cpp
commsgs::geometry_msgs::Twist cmd_vel;
cmd_vel.linear.x = 0.5;   // 前进 m/s
cmd_vel.angular.z = 0.2;  // 旋转 rad/s
```

#### TransformStamped（TF）

```cpp
commsgs::geometry_msgs::TransformStamped tf;
tf.header.stamp = commsgs::builtin_interfaces::Time::Now();
tf.header.frame_id = "map";        // 父坐标系
tf.child_frame_id = "base_link";   // 子坐标系
tf.transform.translation.x = 1.0;
tf.transform.rotation.w = 1.0;
```

### 4.4 构造规划消息

```cpp
commsgs::planning_msgs::Path path;
path.header.stamp = commsgs::builtin_interfaces::Time::Now();
path.header.frame_id = "map";

for (const auto& waypoint : waypoints) {
    commsgs::geometry_msgs::PoseStamped ps;
    ps.header = path.header;
    ps.pose = waypoint;
    path.poses.push_back(ps);
}
```

```cpp
commsgs::planning_msgs::Odometry odom;
odom.header.stamp = commsgs::builtin_interfaces::Time::Now();
odom.header.frame_id = "odom";
odom.child_frame_id = "base_link";
odom.pose.pose.position.x = 0.1;
odom.twist.twist.linear.x = 0.5;
```

### 4.5 构造地图消息

```cpp
commsgs::map_msgs::OccupancyGrid grid;
grid.header.frame_id = "map";
grid.info.resolution = 0.05;   // m/cell
grid.info.width = 100;
grid.info.height = 100;
grid.info.origin.position.x = -2.5;
grid.info.origin.position.y = -2.5;
grid.data.resize(100 * 100, 0);  // 0=自由, 100=占据, -1=未知
```

### 4.6 扩展自定义消息

新增消息需同时维护三层：

1. **Proto 定义** — `autonomy/commsgs/proto/my_msgs.proto`
2. **C++ struct** — `autonomy/commsgs/my_msgs.hpp`
3. **转换函数** — `autonomy/commsgs/my_msgs.cpp` 中实现 `ToProto`/`FromProto`

```protobuf
// my_msgs.proto
syntax = "proto3";
package autonomy.commsgs.proto.my_msgs;
import "autonomy/commsgs/proto/std_msgs.proto";

message MyData {
  std_msgs.Header header = 1;
  float value = 2;
}
```

```cpp
// my_msgs.hpp
namespace autonomy::commsgs::my_msgs {
struct MyData {
    AUTONOMY_SMART_PTR_DEFINITIONS(MyData)
    std_msgs::Header header;
    float value;
};
proto::my_msgs::MyData ToProto(const MyData& data);
MyData FromProto(const proto::my_msgs::MyData& proto);
}
```

重新编译后 `protoc` 自动生成 `my_msgs.pb.h`。

### 4.7 模块集成模式

| 场景 | 使用类型 | 转换时机 |
|------|----------|----------|
| 算法内部 | C++ struct | 无 |
| Autolink Writer/Reader | C++ struct | 框架内部 |
| Autolink Service/Action | proto 类型 | 回调入口 `FromProto`，出口 `ToProto` |
| 持久化 / 日志 | proto 字节流 | 手动 `SerializeToString` |

**PlannerServer 示例**：

```cpp
// 声明：服务使用 proto 类型
using PathValidRequest = commsgs::proto::nav_msgs::IsPathValid_Request;
using PathValidResponse = commsgs::proto::nav_msgs::IsPathValid_Response;

// 回调：proto → C++ → 业务逻辑 → proto
auto path = commsgs::planning_msgs::FromProto(request->path());
response->set_is_valid(server->IsPathValid(path));
```

### 4.8 故障排查

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| 订阅收不到数据 | topic 类型不匹配 | 确认 Writer/Reader 模板参数一致 |
| 字段全为零 | `ToProto` stub 未实现 | 检查对应 `.cpp` 转换函数 |
| TF 变换错误 | `frame_id` / `child_frame_id` 填反 | 对照 [§3.3](03_schema.md#33-stamped-消息模式) |
| 协方差无效 | 矩阵长度不对 | Pose/Twist 需 36 个元素（6×6） |
| proto 编译失败 | import 路径错误 | 使用 `autonomy/commsgs/proto/xxx.proto` 全路径 |
| PointCloud2 迭代越界 | 未先 `resize` | 先用 `PointCloud2Modifier::resize()` |
| 时间戳比较异常 | 混用不同时钟源 | 统一使用 `Time::Now()` |

### 4.9 测试转换往返

```cpp
#include <gtest/gtest.h>
#include "autonomy/commsgs/geometry_msgs.hpp"

TEST(CommsgsTest, PoseStampedRoundTrip) {
    commsgs::geometry_msgs::PoseStamped original;
    original.header.stamp = commsgs::builtin_interfaces::Time(100, 500);
    original.header.frame_id = "test";
    original.pose.position.x = 3.14;

    auto proto = commsgs::geometry_msgs::ToProto(original);
    auto restored = commsgs::geometry_msgs::FromProto(proto);

    EXPECT_EQ(restored.header.frame_id, "test");
    EXPECT_FLOAT_EQ(restored.pose.position.x, 3.14f);
}
```
