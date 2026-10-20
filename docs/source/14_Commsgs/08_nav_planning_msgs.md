# 8. 导航与规划消息

### 8.1 planning_msgs

| 类型 | 字段 | 转换 |
|------|------|------|
| `Path` | header + `poses[]` | — |
| `Odometry` | header、child_frame_id、pose、twist | — |
| `Goals` | header + goals[] | — |

### 8.2 nav_msgs

**C++**：`SpeedLimit`（percentage / speed_limit）。

**Service（proto）**：

| Service | Request → Response |
|---------|-------------------|
| `IsPathValid` | path, max_cost → is_valid, invalid_pose_indices |
| `ClearEntireCostmap` | 空 → 空 |

**Action（proto）**：统一 `Goal`/`Feedback`/`Result`（含 error_code）。

| Action | Goal 要点 |
|--------|-----------|
| NavigateToPose / ThroughPoses | pose(s)、behavior_tree |
| FollowPath | path、controller_id |
| ComputePathToPose / ThroughPoses | start、goal(s)、planner_id |
| SmoothPath | path、smoother_id |
| FollowWaypoints | poses |
| Spin / BackUp / DriveOnHeading / Wait | yaw/x/speed/Duration |
| AssistedTeleop / DummyBehavior | time_allowance / — |

另：`WaypointStatus`。

### 8.3 error_code

仅 proto。常用：`OK=0`、`PLANNING_NO_PATH_FOUND=6007`、`CONTROL_*`、`TASK_*`、`MAP_NOT_READY=7003`。完整见 `error_code.proto`。

### 8.4 vision_msgs

| 类型 | 组成 |
|------|------|
| BoundingBox2D/3D | center + size |
| Detection2D/3D(+Array) | header + bbox + results + id |
| Classification / VisionClass / LabelInfo | class_id、score、name |

### 8.5 visualization_msgs

| 类型 | 说明 |
|------|------|
| `Marker` | ns/id/type/action/pose/scale/color/points/text/lifetime |
| Marker type | ARROW=0 … TEXT_VIEW_FACING=9 |
| 其他 | MarkerArray、InteractiveMarker*、ImageMarker、MeshFile |

ToProto：**stub**。

### 8.6 其他包

| 包 | 类型 | 备注 |
|----|------|------|
| trajectory_msgs | JointTrajectory* | stub |
| diagnostic_msgs | DiagnosticStatus/Array | stub |
| shape_msgs | SolidPrimitive、Mesh、Plane | 命名空间 bug |
| vehicle_msgs / pcl_msgs | 空/占位 | — |

### 8.7 选择与 Action 示例

| 需求 | 消息 |
|------|------|
| 路径 / 单点目标 | Path / PoseStamped |
| 导航 Action | NavigateToPose 等 |
| 检测 / 可视化 | Detection2DArray / Marker |

```cpp
using ActionT = commsgs::proto::nav_msgs::NavigateToPoseAction;
*goal.mutable_pose() = ToProto(target_pose);
client->SendGoal(goal, on_result, on_feedback);
```

### 8.8 参见

- [§3.9 错误码](03_schema.md) · [§4](04_usage.md) · [13 Visualization](../13_Visualization/00_guide.md)
