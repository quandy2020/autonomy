# 5. 任务架构

```text
Bridge / Action Client → TaskServer → Navigator(BT) / 其它 TaskApp → Planner · Controller · Map · TF
```

Bridge、Action、TaskServer、BT、域进程谁调用谁，容易搅在一起。

### 启动

`autolink_launch autonomy.launch` 拉起多进程；各模块 `Configure` → `Start`。  
**无** 进程内 `CreateAutonomy` 聚合入口（已移除）。

### TaskServer 职责

- 注册各 TaskApp（navigation、teleop、…）  
- 调度与互斥（与 Bridge `TaskMuxer` 呼应）  
- 暴露 Action / 内部 GoalChannel 给 Bridge Stub  

BT **不**在 Bridge 里跑；Bridge 只翻译 RPC。

### 状态

`NavigatorInterface` 一类状态：Idle / Running / Completed / Failed / Canceled（见 `interface.hpp`）。

### 常见坑

| 现象 | 说明 |
|------|------|
| 只起 bridge | task/planning/control 未起 |
| 两路 Goal | muxer 应拒第二路；查 BUSY |
| 文档路径 `autonomy/navigator` | 已迁到 `autonomy/task/` |

Bridge 集成：[15 Bridge](../15_Bridge/00_guide.md)。
