# 3. 行为树引擎

**路径：** `autonomy/task/behavior_tree/`（不是已删除的 `autonomy/navigator/` 目录）。

插件 load 失败、tick 行为不对，或要对齐 Nav2 迁移。

BT 在 autonomy/task/；TaskServer 编排。

### 组件

| 组件 | 职责 |
|------|------|
| `BtRunner` / `BtTaskApp` | 加载 XML、驱动 tick、对接 Goal/取消 |
| `bt_node_registry` / `bt_profile` | 插件与默认 profile |
| `BehaviorTreeNavigator` | 导航任务上的 Navigator 壳（`task/navigation/`） |
| `BtContext`（概念上） | planner / controller / TF / options 注入黑板 |

```text
TaskServer → 各 TaskApp（如 navigation）→ BtRunner → Tree + Blackboard + tick
```

### 插件加载

```text
navigator.pb.txt plugin_lib_names
  → AUTONOMY_BT_PLUGIN_PATH（通常 build/lib）
  → dlopen + BT_REGISTER_NODES
  → BehaviorTreeFactory
```

| 失败现象 | 常见原因 |
|----------|----------|
| 9005 / factory 报错 | `.so` 不在 PATH；名字与 `plugin_lib_names` 不一致 |
| XML 解析失败 | 节点 ID 未注册；XML 路径相对 `AUTONOMY_PATH` |

Groot 节点描述：`autonomy/task/conf/behavior_tree/` 下 tree nodes XML（若存在）。

### Goal 与取消

| 阶段 | 行为 |
|------|------|
| 新 Goal | 写黑板 `goal` / `goals`，进入 `RunWithGoal` 类循环 |
| Preempt | 新 Goal 替换（取决于 muxer 策略） |
| Cancel | HALT 运行中 Action 节点 → Canceled 结果 |

Muxer 若报告仍在 navigating，新 Goal 可能被拒——与 Bridge `NAVIGATION_BUSY` 同源逻辑。

### Tick

- 周期：`bt_loop_duration`（常见 10 ms）。  
- 树返回 SUCCESS / FAILURE / 取消 → 映射到 Task/Action 终态。  
- Action 节点：RUNNING 占多 tick；Condition 瞬时 SUCCESS/FAILURE。

### 控制 / 装饰（常用）

| 节点 | 要点 |
|------|------|
| `PipelineSequence` | 前子 RUNNING 时仍可推进后子（流水线） |
| `RecoveryNode` | 主失败 → 恢复子树 → 重试（`number_of_retries`） |
| `RateController` | 限频子树（如 ComputePath 5 Hz） |
| `ReactiveFallback` | 每 tick 从第一个子重新试 |

### 黑板键（常查）

| 键 | 来源 |
|----|------|
| `goal` / `goals` / `path` | Goal、规划结果 |
| `goal_reached_tol`、帧名 | Options / common |
| `selected_planner` 等 | Selector 节点 |

### 与 Nav2

对齐 `nav2_behavior_tree` 语义；Autonomy 可在进程内直调 planner/controller，减少 Action 往返。迁移时核对 topic/服务名与错误码。
