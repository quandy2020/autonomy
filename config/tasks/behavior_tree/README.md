# 行为树 XML 与插件对照

自定义节点在 `autonomy/tasks/behavior_tree/plugins/` 实现，由 `tasks.lua` 的 `TASKS_PLUGIN_LIB_NAMES` 加载。

BT 导航栈（对标 `nav2_bt_navigator`）在 `autonomy/tasks/navigators/` 下，含 `action_type.hpp`（action traits 与 autolink 动作名）。

## 与 nav2 对照

| nav2 | autonomy |
|------|----------|
| `nav2_behavior_tree::BtActionNode` | `behavior_tree::BtActionNode` |
| `nav2_behavior_tree::BtActionServer` | `behavior_tree::BtActionServer` |
| `nav2_behavior_tree::BtCancelActionNode` | `behavior_tree::BtCancelActionNode`（`bt_cancel_action_node.hpp`） |
| `nav2_behavior_tree::BehaviorTreeEngine` | `behavior_tree::BtEngine` |
| `nav2_core::BehaviorTreeNavigator` | `tasks::BehaviorTreeNavigator`（`common/behavior_tree_navigator.hpp`） |
| `nav2_core::NavigatorMuxer` | `tasks::NavigatorMuxer` |
| `nav2_bt_navigator::BtNavigator` | `tasks::BtNavigator` |
| `NavigateToPoseNavigator` | `tasks::NavigateToPoseNavigator` |
| `NavigateThroughPosesNavigator` | `tasks::NavigateThroughPosesNavigator` |

## behavior_tree/ 目录（BT 引擎与插件）

```
behavior_tree/
  bt_engine.*
  bt_action_node.hpp
  bt_action_server.hpp
  bt_service_node.hpp
  bt_cancel_action_node.hpp
  bt_utils.*
  bt_context.hpp
  plugins/
```

入口：`Task` 持有 `BtNavigator`，通过 `BehaviorTreeNavigator` 插件执行 BT。
