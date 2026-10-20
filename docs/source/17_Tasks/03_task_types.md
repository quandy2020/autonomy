# 3. 任务类型

| 任务 | 输入 | BT / 入口 |
|------|------|-----------|
| NavigateToPose | `PoseStamped` | `navigate_to_pose.xml` |
| NavigateThroughPoses | `vector<PoseStamped>` | `navigate_through_poses.xml` |
| FollowPath | 路径 | BT 子节点，非顶层 API |
| ReplanToGoal | 保持 goal | 重请求全局路径 |

流水线：`ComputePath → Smooth → Follow → GoalReached`。

取消 / 超时：`cancel_checker` + `timeout_sec`（接口见 TaskServer / Navigator）。

消息：`nav_msgs.proto` 中 `NavigateToPoseAction` 等 → [14 Commsgs](../14_Commsgs/08_nav_planning_msgs.md)。
