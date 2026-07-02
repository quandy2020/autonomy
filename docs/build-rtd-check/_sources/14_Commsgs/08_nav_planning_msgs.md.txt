# 8. 导航与规划消息

本文详述 `planning_msgs`、`nav_msgs`（proto Action/Service）、`vision_msgs`、`visualization_msgs` 等导航栈上层消息。

## 8.1 planning_msgs

**头文件**：`autonomy/commsgs/planning_msgs.hpp`

导航栈路径与里程计的核心消息包。

### 8.1.1 Path

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs::Header` | 路径坐标系（通常 `"map"`） |
| `poses` | `vector<geometry_msgs::PoseStamped>` | 有序路径点 |

```cpp
commsgs::planning_msgs::Path path;
path.header.stamp = commsgs::builtin_interfaces::Time::Now();
path.header.frame_id = "map";

commsgs::geometry_msgs::PoseStamped wp;
wp.header = path.header;
wp.pose.position.x = 5.0;
wp.pose.orientation.w = 1.0;
path.poses.push_back(wp);
```

**使用方**：`PlannerServer` 输出、`ControllerServer` 输入、可视化模块。

### 8.1.2 Odometry

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs::Header` | `frame_id` = 里程计坐标系（`"odom"`） |
| `child_frame_id` | `string` | 机器人坐标系（`"base_link"`） |
| `pose` | `geometry_msgs::PoseWithCovariance` | 位姿估计 |
| `twist` | `geometry_msgs::TwistWithCovariance` | 速度估计 |

### 8.1.3 Goals

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs::Header` | 目标坐标系 |
| `goals` | `vector<geometry_msgs::PoseStamped>` | 多目标点序列 |

### 8.1.4 转换完整度

| 类型 | ToProto | FromProto |
|------|---------|-----------|
| `Path` | ✅ | ✅ |
| `Odometry` | ⏳ 部分 | ⏳ 部分 |
| `Goals` | ⏳ 部分 | ⏳ 部分 |

## 8.2 nav_msgs

### 8.2.1 C++ 层

**头文件**：`autonomy/commsgs/nav_msgs.hpp`

| 类型 | 说明 |
|------|------|
| `SpeedLimit` | 速度限制（百分比或绝对值） |

```cpp
commsgs::nav_msgs::SpeedLimit limit;
limit.header.frame_id = "base_link";
limit.percentage = true;
limit.speed_limit = 50.0f;  // 50%
```

### 8.2.2 Proto 层 — Service

**头文件**：`autonomy/commsgs/proto/nav_msgs.pb.h`

| Service | Request | Response |
|---------|---------|----------|
| `IsPathValid` | `path`, `max_cost`, `consider_unknown_as_obstacle` | `is_valid`, `invalid_pose_indices` |
| `ClearEntireCostmap` | 空 | 空 |

`IsPathValid` 已在 `PlannerServer` 中使用：

```cpp
namespace nav_proto = commsgs::proto::nav_msgs;
using PathValidRequest = nav_proto::IsPathValid_Request;
using PathValidResponse = nav_proto::IsPathValid_Response;
```

### 8.2.3 Proto 层 — Action

Nav2 行为树 Action 均在 `nav_msgs.proto` 定义，统一结构：

```protobuf
message XxxAction {
  message Goal { ... }
  message Feedback { ... }
  message Result {
    error_code.ErrorCode error_code = 1;
    string error_msg = 2;
  }
}
```

| Action | Goal 关键字段 | Feedback 关键字段 |
|--------|--------------|-------------------|
| `NavigateToPoseAction` | `PoseStamped pose`, `behavior_tree` | `current_pose`, `distance_remaining`, `number_of_recoveries` |
| `NavigateThroughPosesAction` | `repeated PoseStamped poses` | 同上 |
| `FollowPathAction` | `Path path`, `controller_id` | `current_pose`, `distance_to_goal` |
| `ComputePathToPoseAction` | `start`, `goal`, `planner_id` | — |
| `ComputePathThroughPosesAction` | `start`, `goals`, `planner_id` | — |
| `SmoothPathAction` | `Path path`, `smoother_id` | — |
| `FollowWaypointsAction` | `repeated PoseStamped poses` | `current_waypoint_index` |
| `SpinAction` | `target_yaw`, `time_allowance` | `angular_distance_traveled` |
| `BackUpAction` | `target_x`, `speed`, `time_allowance` | `distance_traveled` |
| `DriveOnHeadingAction` | `target_x`, `target_y`, `speed` | `distance_traveled` |
| `WaitAction` | `Duration time` | `time_left` |
| `AssistedTeleopAction` | `Duration time_allowance` | — |
| `DummyBehaviorAction` | — | — |

**Result 统一携带** `error_code.ErrorCode` + `error_msg`，与 [§3.9 错误码分段](03_schema.md#39-错误码分段) 对应。

### 8.2.4 WaypointStatus

| 字段 | 说明 |
|------|------|
| `waypoint_index` | 当前航点索引 |
| `waypoint_status` | 状态枚举 |

## 8.3 error_code

**头文件**：`autonomy/commsgs/proto/error_code.pb.h`

仅 proto 层，无 C++ struct 包装。

### 8.3.1 StatusPb

```protobuf
message StatusPb {
  ErrorCode error_code = 1;
  string msg = 2;
}
```

### 8.3.2 常用错误码

| 错误码 | 值 | 场景 |
|--------|-----|------|
| `OK` | 0 | 成功 |
| `PLANNING_NO_PATH_FOUND` | 6007 | 无可用路径 |
| `CONTROL_NO_VALID_CMD` | 1013 | 控制器无有效命令 |
| `CONTROL_COLLISION` | 1014 | 碰撞检测 |
| `CONTROL_TF_ERROR` | 1011 | TF 变换失败 |
| `TASK_CANCELED` | 9003 | 任务取消 |
| `TASK_PLANNER_FAILED` | 9009 | 规划失败 |
| `TASK_CONTROLLER_FAILED` | 9010 | 控制失败 |
| `MAP_NOT_READY` | 7003 | 地图未就绪 |

完整列表见 `autonomy/commsgs/proto/error_code.proto`。

## 8.4 vision_msgs

**头文件**：`autonomy/commsgs/vision_msgs.hpp`

目标检测与分类消息，对齐 ROS `vision_msgs`。

### 8.4.1 2D 检测

| 类型 | 组成 | 说明 |
|------|------|------|
| `Point2D` | `x, y` (double) | 2D 点 |
| `Pose2D` | `position` + `theta` | 2D 位姿 |
| `BoundingBox2D` | `center` + `size_x/y` | 2D 包围盒 |
| `ObjectHypothesis` | `class_id`, `score` | 分类假设 |
| `Detection2D` | `header` + `bbox` + `results` + `id` | 单个 2D 检测 |
| `Detection2DArray` | `header` + `detections` | 检测数组 |

### 8.4.2 3D 检测

| 类型 | 组成 | 说明 |
|------|------|------|
| `BoundingBox3D` | `center` (Pose) + `size` (Vector3) | 3D 包围盒 |
| `Detection3D` | `header` + `bbox` + `results` + `id` | 单个 3D 检测 |
| `Detection3DArray` | `header` + `detections` | 检测数组 |

### 8.4.3 分类

| 类型 | 说明 |
|------|------|
| `Classification` | `class_id` + `score` |
| `VisionClass` | `class_id` + `class_name` |
| `LabelInfo` | 标签元数据 |
| `VisionInfo` | 视觉模块信息 |

### 8.4.4 使用方

`autonomy/visualization` 模块支持发布 `Detection2DArray`、`BoundingBox3DArray` 等到可视化服务。

## 8.5 visualization_msgs

**头文件**：`autonomy/commsgs/visualization_msgs.hpp`

RViz 风格可视化消息（转换函数为 stub）。

### 8.5.1 Marker

| 字段 | 说明 |
|------|------|
| `header` | 坐标系 |
| `ns` | 命名空间 |
| `id` | 唯一 ID |
| `type` | 形状枚举（ARROW/CUBE/SPHERE/LINE_STRIP 等） |
| `action` | ADD / MODIFY / DELETE |
| `pose` | 位姿 |
| `scale` | 尺寸 (Vector3) |
| `color` | 颜色 (ColorRGBA) |
| `points` | 点集（LINE_STRIP 等） |
| `colors` | 逐点颜色 |
| `text` | TEXT 类型文本 |
| `lifetime` | 显示时长 |

**type 枚举常用值**：

| 值 | 常量 | 形状 |
|----|------|------|
| 0 | `ARROW` | 箭头 |
| 1 | `CUBE` | 立方体 |
| 2 | `SPHERE` | 球体 |
| 3 | `CYLINDER` | 圆柱 |
| 4 | `LINE_STRIP` | 折线 |
| 5 | `LINE_LIST` | 线段列表 |
| 6 | `CUBE_LIST` | 立方体列表 |
| 7 | `SPHERE_LIST` | 球体列表 |
| 8 | `POINTS` | 点云 |
| 9 | `TEXT_VIEW_FACING` | 文字 |

### 8.5.2 其他类型

| 类型 | 说明 |
|------|------|
| `MarkerArray` | Marker 批量 |
| `InteractiveMarker` | 可交互标记 |
| `InteractiveMarkerControl` | 交互控制 |
| `InteractiveMarkerFeedback` | 交互反馈 |
| `MenuEntry` | 右键菜单项 |
| `ImageMarker` | 图像上标记 |
| `MeshFile` | 3D 网格文件 |

## 8.6 其他消息包

### 8.6.1 trajectory_msgs

| 类型 | 说明 |
|------|------|
| `JointTrajectoryPoint` | 关节位置/速度/加速度 |
| `JointTrajectory` | 关节轨迹序列 |
| `MultiDOFJointTrajectoryPoint` | 多自由度轨迹点 |
| `MultiDOFJointTrajectory` | 多自由度轨迹 |

> cpp 转换函数为空 stub，适用于机械臂场景。

### 8.6.2 diagnostic_msgs

| 类型 | 说明 |
|------|------|
| `KeyValue` | 键值对 |
| `DiagnosticStatus` | 诊断状态（OK/WARN/ERROR/STALE） |
| `DiagnosticArray` | 诊断数组 |

### 8.6.3 shape_msgs

| 类型 | 说明 |
|------|------|
| `SolidPrimitive` | 基本几何体（BOX/SPHERE/CYLINDER/CONE/PLANE） |
| `Mesh` | 三角网格 |
| `Plane` | 平面 |
| `MeshTriangle` | 三角面片 |

> 注意：当前 hpp 中 struct 命名空间有误，应为 `shape_msgs` 而非 `builtin_interfaces`。

### 8.6.4 vehicle_msgs

| 类型 | 说明 |
|------|------|
| `RobotEvent` | 机器人事件（空 struct，待扩展） |
| `RobotState` | 机器人状态（空 struct，待扩展） |

### 8.6.5 pcl_msgs

PCL 相关消息，当前为空 namespace，预留扩展。

## 8.7 导航栈消息依赖图

```
                    nav_msgs.proto (Action/Service)
                           │
              ┌────────────┼────────────┐
              ▼            ▼            ▼
      planning_msgs   geometry_msgs   error_code
              │            │
              ▼            ▼
           Path      PoseStamped / Twist
              │            │
              ▼            ▼
         Controller    Transform
              │            │
              ▼            ▼
          cmd_vel       TF tree
```

## 8.8 Action 使用模式（Autolink）

```cpp
#include "autonomy/commsgs/proto/nav_msgs.pb.h"

using ActionT = commsgs::proto::nav_msgs::NavigateToPoseAction;
using Goal = ActionT::Goal;
using Feedback = ActionT::Feedback;
using Result = ActionT::Result;

auto client = node->CreateActionClient<ActionT>("navigate_to_pose");

Goal goal;
*goal.mutable_pose() = commsgs::geometry_msgs::ToProto(target_pose);
goal.set_behavior_tree("navigate_to_pose_w_replanning.xml");

client->SendGoal(goal,
    [](const Result& result) {
        if (result.error_code() != commsgs::proto::error_code::OK) {
            // 处理错误
        }
    },
    [](const Feedback& fb) {
        // distance_remaining = fb.distance_remaining()
    });
```

## 8.9 消息选择指南

| 需求 | 推荐消息 | 包 |
|------|----------|-----|
| 全局路径 | `Path` | planning_msgs |
| 单点目标 | `PoseStamped` | geometry_msgs |
| 多航点导航 | `NavigateThroughPosesAction` | nav_msgs proto |
| 路径跟踪 | `FollowPathAction` | nav_msgs proto |
| 路径有效性检查 | `IsPathValid` service | nav_msgs proto |
| 速度限制 | `SpeedLimit` | nav_msgs |
| 目标检测可视化 | `Detection2DArray` | vision_msgs |
| 路径可视化 | `Marker` (LINE_STRIP) | visualization_msgs |
| 操作结果 | `error_code.ErrorCode` | error_code proto |
