# Navigator 配置

`navigator.lua` 由 `navigator::LoadOptions` 解析，字段与 `navigator/proto/navigator_options.proto` 对应。

## 主要字段

| 字段 | 说明 |
|------|------|
| `global_frame` / `robot_base_frame` | 导航坐标系（默认来自 `common.lua`） |
| `default_planner_id` / `default_controller_id` / `default_goal_checker_id` / `default_smoother_id` | BT Selector 与 Action 默认插件 |
| `bt_loop_duration` / `default_server_timeout` | 行为树 tick 与服务超时（毫秒） |
| `plugin_lib_names` | BT 插件库名列表（须与 CMake 目标一致） |
| `navigate_to_pose` / `navigate_through_poses` | 各导航器的 BT XML 与 enable 开关 |

`odom_topic` 由 `common.lua` 供 controller 等模块使用，不属于 `NavigatorOptions`。

## 加载链

```
autonomy.lua → include navigator/navigator.lua
system::CreateOptions → navigator::LoadOptions
```
