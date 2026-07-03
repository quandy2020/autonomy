# 3. 任务类型

### 3.1 NavigateToPose（单点导航）

从机器人当前位姿导航到单个目标位姿。

| 属性 | 说明 |
|------|------|
| 输入 | `PoseStamped goal` |
| 输出 | 导航结果（成功/失败/取消） |
| BT XML | `config/navigator/behavior_tree/navigate_to_pose.xml` |
| Navigator 名 | `navigate_to_pose` |

典型流程（设计目标）：

```
ComputePathToPose → SmoothPath → FollowPath → GoalReached
```

### 3.2 NavigateThroughPoses（多点导航）

依次经过多个航点，到达最后一个目标。

| 属性 | 说明 |
|------|------|
| 输入 | `vector<PoseStamped> goals` |
| BT XML | `navigate_through_poses.xml` |
| Navigator 名 | `navigate_through_poses` |

### 3.3 FollowPath（路径跟踪）

由 Controller 执行的子任务，通常由 BT 节点 `FollowPath` 触发，不直接暴露为顶层用户 API。

### 3.4 ReplanToGoal（重规划）

保持目标不变，重新请求全局路径。适用于动态障碍或路径失效场景。

```cpp
autonomy->ReplanToGoal(goal);
```

### 3.5 取消与超时

`NavigateToPose` 接受 `cancel_checker` 回调与 `timeout_sec`：

```cpp
std::atomic<bool> canceled{false};
auto checker = [&]() { return canceled.load(); };

bool ok = autonomy->NavigateToPose(goal, checker, /*keep_alive=*/true, 60.0);
```

### 3.6 消息定义

Action 类型定义于 `autonomy/commsgs/proto/nav_msgs.proto`：

- `NavigateToPoseAction`
- `NavigateThroughPosesAction`
- `FollowPathAction`

详见 [14 Commsgs · 导航消息](../14_Commsgs/08_nav_planning_msgs.md)。

### 3.7 相关文档

- [§7 API 参考](07_api_reference.md)
- [16 Navigator · BT 引擎](../16_Navigator/03_bt_engine.md)
