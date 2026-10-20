# 8. 导航与规划消息

发 Path、调 NavigateToPose Action，或解析 error_code。

planning_msgs 承载路径与里程计；nav 相关 **Action/Service 在 proto**，C++ 侧常用 `commsgs::proto::nav_msgs::*`。

| 需求 | 消息 |
|------|------|
| 显示/检查路径 | Path |
| BT 导航 | NavigateToPose Action |
| 检测框 | Detection2DArray |
| RViz Marker | 先确认 visualization 转换是否 stub |

### planning_msgs

| 类型 | 组成 | 转换 |
|------|------|------|
| `Path` | header + poses[] | 完整 |
| `Odometry` | header, child_frame_id, pose, twist | 完整 |
| `Goals` | header + goals[] | 视实现 |

### nav_msgs

**C++ struct：** 主要是 `SpeedLimit`（percentage / speed_limit）。

**Service（proto）：** 如 `IsPathValid`（path + max_cost → is_valid）。

**Action（proto）：** 统一 Goal / Feedback / Result（含 error_code）。

| Action | Goal 要点 |
|--------|-----------|
| NavigateToPose / ThroughPoses | pose(s)、behavior_tree |
| FollowPath | path、controller_id |
| ComputePathToPose / ThroughPoses | start、goal(s)、planner_id |
| Spin / BackUp / Wait 等 | 各行为参数字段 |

### error_code

定义在 `error_code.proto`（仅 proto）。例：`OK=0`、`PLANNING_NO_PATH_FOUND=6007`、`MAP_NOT_READY=7003`。Action Result 里常带错误码字段。

### vision_msgs / visualization_msgs

| 包 | 说明 |
|----|------|
| vision_msgs | Detection2D/3D、BoundingBox*；Perception base 输出相关 |
| visualization_msgs | Marker 等；**ToProto 多为 stub**，见 [§9](09_survey.md) |

### 其他包（低优先）

trajectory_msgs、diagnostic_msgs：stub。shape_msgs：命名空间历史问题。vehicle_msgs / pcl_msgs：占位。

### Action 调用示例

```cpp
using ActionT = commsgs::proto::nav_msgs::NavigateToPoseAction;
*goal.mutable_pose() = ToProto(target_pose);
client->SendGoal(goal, on_result, on_feedback);
```

### 常见坑

| 现象 | 说明 |
|------|------|
| Action 编译不过 | 包含 `proto/nav_msgs.pb.h` |
| Result 错误码未读 | 看 Result proto 字段 |
| Path frame 错 | header.frame_id 应为 map 等 |

### 延伸阅读

[§3 错误码段](03_schema.md) · [13 Visualization](../13_Visualization/00_guide.md)
