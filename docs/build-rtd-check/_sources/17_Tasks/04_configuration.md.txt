# 4. 任务配置

导航任务配置集中在 `config/navigator/`，通过 Lua 加载为 `NavigatorOptions`。

### 4.1 配置链

```
config/autonomy.lua
    └── include "navigator/navigator.lua"
            └── NavigatorOptions (Protobuf)
                    └── BehaviorTreeNavigator / BtNavigator
```

### 4.2 主配置文件

`config/navigator/navigator.lua` 关键字段：

| 字段 | 说明 |
|------|------|
| `bt_loop_duration` | BT tick 周期（ms） |
| `default_server_timeout` | Action 超时 |
| `navigate_to_pose` | 单点导航 BT 配置 |
| `navigate_through_poses` | 多点导航 BT 配置 |
| `plugin_lib_names` | BT 插件库列表（52 个） |

### 4.3 与 common.lua 共享参数

以下参数须与 `config/common.lua` 保持一致：

| 参数 | 使用方 |
|------|--------|
| `global_frame` | navigator, planner, controller |
| `robot_base_frame` | 同上 |
| `goal_reached_tolerance` | controller, navigator |
| `default_planner_id` | planner, navigator |
| `default_controller_id` | controller, navigator |

### 4.4 BT XML 路径

| 任务 | 默认 XML |
|------|----------|
| 单点导航 | `config/navigator/behavior_tree/navigate_to_pose.xml` |
| 多点导航 | `config/navigator/behavior_tree/navigate_through_poses.xml` |

### 4.5 兼容垫片 `task_options.lua`

历史文件 `config/task_options.lua` 仅 `include "navigator/navigator.lua"`，**已废弃**，请直接使用 `navigator.lua`。

### 4.6 环境变量

| 变量 | 说明 |
|------|------|
| `AUTONOMY_BT_PLUGIN_PATH` | BT 插件 `.so` 搜索路径（冒号分隔） |

### 4.7 相关文档

- 仓库内配置文件：``config/navigator/README.md``（行为树 XML 与 `navigator.lua` 说明）
- [16 Navigator · 使用指南](../16_Navigator/04_usage.md)
