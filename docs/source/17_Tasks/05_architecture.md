# 5. 任务架构

```text
Bridge/Action → TaskServer → Navigator/BT → Planner / Controller / Map / TF
```

启动：`autolink_launch autonomy.launch` → 各进程 Configure/Start → 发令。 
进程内 `CreateAutonomy` 已移除。

| 状态 | 说明 |
|------|------|
| Idle / Running / Completed / Failed / Canceled | `NavigatorInterface` |

互斥：同一时刻通常一个导航类任务（Muxer）。Bridge 集成见 [15](../15_Bridge/00_guide.md)。
