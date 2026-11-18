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

### 原生 GetPlan 图证据扩展

现有 `automsgs/proto/srvs/get_plan.proto` 保留 Request 的 start/goal/tolerance 和
Response 的 plan 字段编号。新增 planner_id、request_id、evidence_only、timeout_ms
和 require_graph_evidence；响应回传 request_id、实际插件 id、明确 Status、
长度、grid fallback 标志及 GraphEvidence。几何仍使用现有 PoseStamped/Path。

GraphEvidence 明确保存 frame_id、map_epoch、geometry_revision、诊断水位和
两种传感器各自的 ObservationPrefix。每个前缀包含 present 位掩码与独立
lidar_sequence/rgbd_sequence；observation_baseline 和 recovery generation
来自实际图 owner，不由客户端或时间戳制造。

`UNSPECIFIED=0` 不表示成功。要求图证据时，SUCCESS 必须具备完整匹配的证据；NO_ROUTE
可以返回源证据，但没有可执行路径。需要图证据的客户端拒绝只支持旧 plan
字段的服务器。所有请求结果通过精确 token 和插件 id 匹配，不把版本编码到
frame_id、时间戳或 error_msg。客户端还验证路径长度、端点、朝向与容量。

生成后的 C++ 包含 `<automsgs/srvs/get_plan.pb.h>`，使用
`automsgs::srvs::GetPlan::Request/Response`；传输为原生 Autolink Service/Client。
新增字段需要重新生成消息并部署相容的 PlannerServer 和任务客户端。

原生 Client 的 `RetransmitRequest` 只重发仍未完成的同一 shared request，
保留原 sequence 与 promise。它是显式启用的幂等查询能力；路线客户端在
原始 deadline 内使用，超时后保留单个 pending slot 等待真正的迟到回复。
此能力不自动应用于导航动作或其他有副作用的服务。

传输回调的生命周期独立于消息协议。原生 Client/Service 每次初始化使用
新的回调世代，销毁时关闭入口并等待已接收的工作退出。Service 在实际
worker 退出后撤销拓扑端点，保留已进入请求发送回复的机会。接收器的普通和
历史监听器持有可关闭的共享状态，分发器复制监听器后仍能安全处理
接收器退役。历史到达标志不引用线程栈，也不需要为此扩展 automsgs。

路线调用和错误行为见 [VistaPlanner](../08_Planning/07_vista_planner.md)。

### 延伸阅读

[§3 错误码段](03_schema.md) · [13 Visualization](../13_Visualization/00_guide.md)
