# tasks 配置

`tasks.lua` 由 `tasks::LoadTaskOptions` 解析，字段与 `tasks/proto/task_options.proto` 对应。

## 文件

| 文件 | 说明 |
|------|------|
| `tasks.lua` | Task / BT 运行时与插件列表 |
| `behavior_tree/*.xml` | 行为树 |
| `behavior_tree/README.md` | XML 节点与 C++ 插件对照 |

## 参数说明

| 字段 | 来源 | 用途 |
|------|------|------|
| `global_frame` / `robot_base_frame` | `common.lua` | TF、规划、BT blackboard |
| `default_planner_id` / `default_controller_id` / `default_goal_checker_id` / `default_smoother_id` | `common.lua` | BT Selector 与 Action 默认插件 |
| `goal_reached_tolerance` | `common.lua` | BT `GoalReached` 与 controller goal checker |
| `bt_loop_duration` | 本文件 | BT 主循环周期 (ms) |
| `default_server_timeout` | 本文件 | blackboard `server_timeout` (ms) |
| `local_survival_timeout` | 本文件 | `navigate_to_pose.xml` 局部生存超时 (s) |
| `plugin_lib_names` | 本文件 | 动态加载的 BT 插件 `.so` 名 |
| `plugin_lib_path` | 本文件 | 插件路径；空则用 `AUTONOMY_BT_PLUGIN_PATH` / install lib |
| `navigate_to_pose` / `navigate_through_poses` | 本文件 | `enable` + `behavior_tree_file` |
| `enable_autolink_action_servers` | 本文件 | 是否挂载 autolink Action 服务 |

路径 blackboard 固定为 `goal` / `goals` / `path`（见 `bt_context.hpp`），不在 Lua 中配置。

`odom_topic` 由 `common.lua` 供 controller 等模块使用，不属于 TaskOptions。

## 修改 BT 插件

增删 XML 节点时，同步修改 `plugin_lib_names` 与 `behavior_tree/plugins/`（见 `behavior_tree/README.md`）。

## 修改默认行为树

改 `navigate_to_pose.behavior_tree_file` 或 `navigate_through_poses.behavior_tree_file`。

## 加载链

```text
autonomy.lua → include tasks/tasks.lua
system::CreateOptions → tasks::LoadTaskOptions
```
