# 3. 任务类型

| 任务 | 输入 | BT / 入口 |
|------|------|-----------|
| NavigateToPose | 单个 `PoseStamped` | `navigate_to_pose.xml` |
| NavigateThroughPoses | 多个 pose | `navigate_through_poses.xml` |
| FollowPath | 已有 path | BT 内 FollowPath 节点，**不是** Bridge 顶层 RPC |
| ReplanToGoal | 保持 goal | Replan RPC / BT 内重规划 |
标准流水线（概念）：`ComputePath → Smooth → Follow → GoalReached`（单点树可能先 GoalReached）。

**什么时候看：** 选 Action/RPC、改 BT XML 或查取消语义前，先认清「顶层任务」有哪些。

### 其它 TaskApp

建图、遥操、回充、探索等由 `TaskServer` 按 `TaskAppOptions` 开关挂载，实现分散在 `autonomy/task/` 各子目录。Bridge 域服务与之一一对应（见 [15 rpcs/02](../15_Bridge/rpcs/02_service_overview.md)）。

### 取消与超时

- 取消：`cancel_checker`、BT HALT、Bridge `Cancel` / `CancelAllGoals`。  
- 超时：`timeout_sec`、Navigate `options.timeout_seconds` 等，见 conf 与 proto。

消息定义：`commsgs` / `automsgs` 中 `NavigateToPoseAction` 等 → [14 Commsgs](../14_Commsgs/08_nav_planning_msgs.md)。

### 常见坑

| 坑 | 说明 |
|----|------|
| 搜到 `common::Task` | 线程池工作项，不是导航任务 |
| FollowPath 当顶层 API | 应 Navigate + 树内 Follow |
| 多点用单点 XML | 航点逻辑不对，换 through_poses |
