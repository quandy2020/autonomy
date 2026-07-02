# 6. 执行模式

Autonomy 支持两种导航执行路径：**行为树（BT）模式**与**直驱（Direct）模式**。

### 6.1 模式对比

| 维度 | BT 模式 | 直驱模式 |
|------|---------|----------|
| 开关 | `use_bt_navigation = true` | `use_bt_navigation = false` |
| 编排 | BT XML + 52 插件节点 | `Autonomy` 内直接调用 Server |
| 恢复行为 | 支持（Replan、Spin 等） | 不支持 |
| 当前默认 | ⏳ 待恢复 | ✅ 当前生效 |

### 6.2 BT 模式（设计目标）

```
NavigateToPose
  → BtNavigator
  → navigate_to_pose.xml
  → ComputePathToPose → FollowPath → GoalReached
```

需要：

- `BtEngine` / `BtActionServer` 实现
- BT 插件 `.so` 在 `AUTONOMY_BT_PLUGIN_PATH`
- `Configure()` 中 `use_bt_navigation_ = true`

### 6.3 直驱模式（当前实现）

```
NavigateToPose
  → NavigateDirectToPose()
  → PlannerServer::GetPlan()
  → NotifyPath()（不自动 FollowPath）
```

`Autonomy::Configure()` 当前强制 `use_bt_navigation_ = false`，日志输出 `direct planner wiring`。

### 6.4 nav_test 中的 `--use_bt`

```bash
# BT 模式（需 BT 栈完整）
./build/bin/autonomy_nav_test --use_bt=true ...

# 直驱模式
./build/bin/autonomy_nav_test --use_bt=false ...
```

> 注意：即使传入 `--use_bt=true`，若 `Configure()` 强制关闭 BT，实际仍可能走直驱路径。以源码行为为准。

### 6.5 选型建议

| 场景 | 建议 |
|------|------|
| 验证全局规划 | 直驱模式 + `GetPlan` |
| 完整导航闭环 | BT 模式（待栈恢复） |
| 对比 Nav2 行为 | BT 模式 + 相同 XML |

### 6.6 相关文档

- [18 Tools · nav_test](../18_Tools/04_nav_test.md)
- [16 Navigator · BT 引擎](../16_Navigator/06_bt_engine.md)
