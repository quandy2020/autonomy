(tasks-overview)=
# 1. 任务概览

导航任务入口进程：**`autonomy.task`（`TaskServer`）**。上游：Bridge gRPC、Action Client；下游：BT → planning / control / map。
```text
Bridge / Action → TaskServer → Navigator(BT) → planning / control / map
```
BT 树与插件：[16 Navigator](../16_Navigator/00_guide.md)。  
Bridge RPC：[15 Bridge](../15_Bridge/00_guide.md)（需 `BUILD_GRPC=ON`）。

需要分清 TaskServer、Navigator(BT)、Bridge：谁管发令、谁管走路径。

先确认 task 与 planning/control 都在跑，帧名与默认 planner/controller id 与 08/09 章一致，再改树或 RPC。

### 常见顶层任务

| 任务 | 含义 |
|------|------|
| NavigateToPose | 单点 |
| NavigateThroughPoses | 多点 |
| 遥操 / 回充 / 建图 / 探索 | `autonomy/task/` 下各 TaskApp，由 options 开关 |

配置、取消、超时：[04](04_configuration.md)、[06](06_execution_modes.md)、[03](03_task_types.md)。

### 别混淆

- `common::Task`：线程池工作项，**不是**导航任务。  
- `CreateAutonomy`：已移除；用 `autolink_launch autonomy.launch`。  
- BT 代码路径：`autonomy/task/behavior_tree/`、`autonomy/task/navigation/`（文档章名仍叫 Navigator）。
