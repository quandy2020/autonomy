# 2. 快速开始

### 2.1 多进程栈 + 发令（推荐）

```bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
export AUTONOMY_BT_PLUGIN_PATH="$PWD/build/lib"
autolink_launch autonomy.launch
```

主栈不会自动下发目标。请用 **Bridge**、autolink Action Client 或上层业务向 `autonomy.task` 发令。见 [04 Running](../04_Running/02_quickstart.md)。

> 离线 `autonomy_nav_test` **已移除**。

### 2.2 TaskServer 进程

`autonomy/task/task_main.cpp` 构造 `TaskServer`，加载 task / navigator conf 后进入 `WaitForShutdown`。共享栈 conf 仍可用 `system::CreateOptions("autonomy.pb.txt")` 中的相关子字段（若模块接入）。

### 2.3 配置前提

- BT XML：`autonomy/task/conf/behavior_tree/`
- Navigator：`autonomy/task/conf/navigator.pb.txt`
- 共享快照：`autonomy/system/conf/autonomy.pb.txt`

### 2.4 下一步

| 目标 | 文档 |
|------|------|
| 任务类型说明 | [§3 任务类型](03_task_types.md) |
| 修改 BT 配置 | [§4 任务配置](04_configuration.md) |
| 理解执行路径 | [§6 执行模式](06_execution_modes.md) |
