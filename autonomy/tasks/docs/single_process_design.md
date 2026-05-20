# 单进程方案说明

## 背景

原设计对齐 Nav2 BT Navigator：行为树节点通过 **Action / Service / Topic** 与 `PlannerServer`、`ControllerServer` 等通信，并依赖 Autolink 中间件。当前工程目标为：

- **CMake + C++17** 构建 `libautonomy`
- **不依赖 Autolink / Boost**（已由 `std::` 替代项完成基础迁移）
- **tasks 模块单进程调度**：BT 插件直接调用进程内的 server API

## 方案原则

1. **最小中间层**：去掉 ActionClient / ServiceClient 假异步线程（`detach` 立即 SUCCESS）对关键路径的影响，改为显式 `BtStatefulActionNode`。
2. **单一事实来源**：`TaskContext` 持有 planner、controller、costmap、tf；blackboard 只存 `shared_ptr<TaskContext>` 与 BT 端口数据。
3. **XML 仍为权威**：`config/tasks/behavior_tree/navigate_to_pose.xml` 描述导航策略，代码负责把 XML 端口映射到进程内 API。
4. **渐进迁移**：尚未改造的节点可暂时保留 `BtActionNode` / `BtServiceNode` 桩，保证 Recovery 分支可跑通。

## 分层设计

```
┌─────────────────────────────────────────────────────────┐
│  Application (main, 未来 TaskManager API)                │
├─────────────────────────────────────────────────────────┤
│  TaskScheduler — 配置、生命周期、NavigateToPose API       │
├─────────────────────────────────────────────────────────┤
│  NavigateToPoseNavigator — goal 变换、BT 加载、feedback   │
├─────────────────────────────────────────────────────────┤
│  BtActionServer + BehaviorTreeEngine — tick 循环         │
├─────────────────────────────────────────────────────────┤
│  BT Plugins — 读 blackboard / TaskContext，调 server     │
├─────────────────────────────────────────────────────────┤
│  PlannerServer │ ControllerServer │ Costmap │ TF         │
└─────────────────────────────────────────────────────────┘
```

## 已完成的进程内改造

| 能力 | 实现位置 | 说明 |
|------|----------|------|
| 路径规划 | `ComputePathToPoseAction` | `PlannerServer::GetPlan()` + `CancelChecker` |
| 路径跟踪 | `FollowPathAction` | `BeginFollowPath` / `TickFollowPath` / `EndFollowPath` |
| 代价地图清理 | `Clear*Costmap*` | `utils/costmap_clear_utils` |
| 路径有效性 | `IsPathValidCondition` | `utils/path_validation_utils` |
| 等待 | `WaitAction` | 进程内计时，匹配 XML `wait_duration` |
| Planner 别名 | `utils/planner_id_utils` | `GridBased` → `navfn_planner` |
| 黑板初始化 | `SetupNavigateToPoseBlackboard` | 对齐 `navigate_to_pose.xml` 端口 |

## ControllerServer 跟踪 API

为支持 BT 每 tick 推进一步控制（而非阻塞式 action），在 `ControllerServer` 上增加：

```cpp
bool BeginFollowPath(path, controller_id, goal_checker_id, progress_checker_id);
FollowPathTickResult TickFollowPath(cancel_checker);
void EndFollowPath();
```

`FollowPathAction::onRunning()` 每次 BT tick 调用一次 `TickFollowPath`；取消或 halt 时 `EndFollowPath()`。

> 注：当前 `ComputeAndPublishVelocity()` 等仍为占位实现，接口已对齐 BT 调度节奏，待 control 模块补全具体控制律。

## 取消语义

- `TaskScheduler::RequestCancel()` → `cancel_requested_ = true` + `BtActionServer::RequestCancel()`
- `TaskContext::CancelChecker()` 供 `GetPlan` / `TickFollowPath` 查询
- `BtStatefulActionNode::isCancelRequested()` 在 tick 开头失败返回

## 配置约定

### tasks.lua

- `plugin_lib_names`：须包含 XML 中用到的所有 BT 插件动态库（见仓库 `config/tasks/tasks.lua`）。
- `navigators` 含 `"navigate_to_pose"` 时，即使未写 `navigate_to_pose.enable = true` 也会注册 Navigator。

### 默认行为树

- 代码默认：`navigate_to_pose.xml`
- 与 Nav2 完整恢复链不同的精简版见同目录其他 XML（如 `navigate_to_pose_w_replanning_and_recovery.xml`）。

## 待办（可选后续）

- [ ] `DriveOnHeading` / `Spin` / `BackUp` 改为进程内运动原语
- [ ] `ReinitializeGlobalLocalization` 对接定位模块 API
- [ ] Selector 节点完全去掉 `topic_name` 占位逻辑
- [ ] `ControllerServer` 实现真实速度发布与 goal/progress checker
- [ ] 统一 `local_costmap` 与 controller 侧 costmap 实例

## 相关代码索引

| 主题 | 路径 |
|------|------|
| 调度器 | `scheduler/task_scheduler.*` |
| 共享上下文 | `common/task_context.hpp` |
| 黑板 | `common/bt_blackboard_setup.*` |
| Stateful 基类 | `behavior_tree/bt_stateful_action_node.hpp` |
| 入口 | `tasks/main.cpp` |
