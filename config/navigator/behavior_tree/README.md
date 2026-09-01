# 行为树（BT）

自定义节点在 `autonomy/task/navigation/plugins/` 实现，由 `navigator.lua` 的 `plugin_lib_names` 加载。

BT 导航栈（对标 `nav2_bt_navigator`）在 `autonomy/task/navigation/` 下。

## 与 nav2 对照

| nav2 | autonomy |
|------|----------|
| `nav2_core::BehaviorTreeNavigator` | `navigator::BehaviorTreeNavigator`（`common/behavior_tree_navigator.hpp`） |
| `nav2_core::NavigatorMuxer` | `navigator::NavigatorMuxer` |
| `nav2_bt_navigator::BtNavigator` | `navigator::BtNavigator` |
| `NavigateToPoseNavigator` | `navigator::NavigateToPoseNavigator` |
| `NavigateThroughPosesNavigator` | `navigator::NavigateThroughPosesNavigator` |
