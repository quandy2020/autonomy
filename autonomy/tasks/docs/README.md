# autonomy.tasks 文档

本目录描述 **单进程行为树导航** 的方案、架构与运行方式，对应代码根目录 `autonomy/tasks/`。

## 文档索引

| 文档 | 说明 |
|------|------|
| [architecture.md](architecture.md) | 总体架构、模块划分、调用关系 |
| [single_process_design.md](single_process_design.md) | 单进程方案背景、与 ROS/Autolink 差异、迁移要点 |
| [navigate_to_pose_execution.md](navigate_to_pose_execution.md) | `navigate_to_pose.xml` 执行流程与节点映射 |
| [blackboard_and_plugins.md](blackboard_and_plugins.md) | Blackboard 键、BT 插件类型（Stateful / Action 桩 / Service 桩） |

## 快速入口

- **可执行程序**：`autonomy/tasks/main.cpp` → `TaskScheduler`
- **默认行为树**：`config/tasks/behavior_tree/navigate_to_pose.xml`
- **任务配置**：`config/tasks/tasks.lua`
- **规划 / 控制配置**：`config/planner/planner.lua`、`config/control/controller.lua`

## 运行示例

在 **Docker / Linux** 中构建时，请使用容器内路径（不要用 macOS 的 `/Users/...`）：

```bash
./build/bin/autonomy_tasks_main \
  --configuration_directory=/workspace/autonomy/src/autonomy/config \
  --run_navigate_to_pose \
  --nav_goal_x=2.0 \
  --nav_goal_y=1.0 \
  --nav_goal_yaw=0.0
```

`--configuration_directory` 可省略：会依次尝试该路径、`{源码}/config`、安装目录 `share/autonomy/config`。若传入的路径在容器内不存在，会自动回退并打印 `Using configuration directory: ...`。

行为树插件需先编译为独立 `.so`（`ninja behavior_tree_plugins` 会随主目标一起构建），默认从 `tasks.lua` 的 `plugin_lib_path`（如 `build/lib`）加载；也可设置环境变量 `AUTONOMY_BT_PLUGIN_PATH`。

按 `Ctrl+C` 会触发 `TaskScheduler::RequestCancel()`，行为树在下一 tick 收到取消并退出。

## 目录结构（代码）

```
tasks/
├── main.cpp                 # 进程入口
├── scheduler/               # TaskScheduler：持有 planner/controller/navigator
├── common/                  # TaskContext、BehaviorTreeNavigator、黑板初始化
├── navigator/               # NavigateToPoseNavigator 等
├── behavior_tree/           # BtActionServer、Engine、插件
│   └── plugins/             # BT 节点（action/condition/control/decorator）
├── utils/                   # costmap 清理、路径校验、planner id 映射
└── docs/                    # 本文档目录
```
