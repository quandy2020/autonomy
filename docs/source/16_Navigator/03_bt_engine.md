# 3. 行为树引擎

路径：`autonomy/navigator/behavior_tree/`。实现对齐 nav2；部分实现演进中。

### 3.1 组件

| 组件 | 职责 |
|------|------|
| `BtEngine` | `dlopen` 插件、`createTreeFromFile` |
| `BtActionServer` | Goal/Preempt/Cancel + tick |
| `BtContext` | planner / controller / tf / options |
| `BehaviorTreeNavigator` | Navigator 模板基类 |

```text
BtNavigator → BtEngine + BtContext + BehaviorTreeNavigator
└── BtActionServer → Tree + Blackboard + tick(10ms)
```

### 3.2 插件加载

```text
navigator.lua plugin_lib_names → LoadPlugins(AUTONOMY_BT_PLUGIN_PATH)
→ BT_REGISTER_NODES(factory) → BehaviorTreeFactory
```

解析失败 → 错误码 9005。Groot：`autonomy_tree_nodes.xml`。

### 3.3 BtActionServer

| 阶段 | 方法 |
|------|------|
| 配置 / 激活 | `OnConfigure` / `OnActivate` |
| 运行 | `RunWithGoal`（阻塞至终态） |
| 取消 | `Cancel` → HALT |

回调：Goal（写黑板）· Loop（Feedback）· Preempt · Completed（Result）。 
Muxer：`IsNavigating()` 则拒绝新 Goal。

### 3.4 Tick 与状态

周期：`bt_loop_duration`（默认 10 ms）。树 SUCCESS/FAILURE/CANCEL → `RunStatus` → Action Result。

| 节点类 | 返回 |
|--------|------|
| Action | RUNNING / SUCCESS / FAILURE |
| Condition | SUCCESS / FAILURE（瞬时） |

### 3.5 控制 / Decorator（摘要）

| 节点 | 要点 |
|------|------|
| `PipelineSequence` | 前子 RUNNING 时可推进后子 |
| `RecoveryNode` | 主失败 → 恢复子 → 重试（`number_of_retries`） |
| `RateController` | 限频（如 ComputePath 5 Hz） |

### 3.6 黑板键（常用）

| 键 | 来源 |
|----|------|
| `goal` / `path` / `goals` | Goal / 规划 |
| `goal_reached_tol` / 帧名 | Options 注入 |
| `selected_planner` 等 | Selector 节点 |
| `autolink_node` | 可选 |

### 3.7 与 Nav2

对齐 `nav2_behavior_tree`；Autonomy 插件可经 `BtContext` 进程内调用（减少 Action 往返）。迁移时核对端口名与错误码。
