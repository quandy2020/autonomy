# 6. 执行模式

Autonomy 支持两种导航执行路径：**行为树（BT）模式**与较简的直驱/服务调用路径（演进中）。入口均为多进程 `TaskServer`，而非进程内聚合。

### 6.1 模式对比

| 维度 | BT 模式 | 直驱 / 服务路径 |
|------|---------|-----------------|
| 编排 | BT XML + 插件节点 | 直接调用 Planner / Controller |
| 恢复行为 | 支持（Replan、Spin 等） | 有限 |
| 推荐验证 | launch + Bridge | 单模块调试 |

### 6.2 BT 模式

```
Action / Bridge
  → TaskServer
  → navigate_to_pose.xml 等
  → ComputePathToPose → FollowPath → GoalReached
```

需要：

- BT 插件 `.so` 在 `AUTONOMY_BT_PLUGIN_PATH`
- XML 位于 `autonomy/task/conf/behavior_tree/`

### 6.3 多进程联调

```bash
autolink_launch autonomy.launch
# 另开客户端：Bridge 或 autolink Action
```

> 离线 `autonomy_nav_test --use_bt=…` **已移除**。

### 6.4 选型建议

| 场景 | 建议 |
|------|------|
| 验证全局规划 | planning 进程 + 服务/Action |
| 完整导航闭环 | task + planning + control + Bridge |
| 对比 Nav2 行为 | BT XML + 相同语义插件 |

### 6.5 相关文档

- [04 Running · 多进程栈](../04_Running/03_autonomy_process.md)
- [16 Navigator · BT 引擎](../16_Navigator/03_bt_engine.md)
