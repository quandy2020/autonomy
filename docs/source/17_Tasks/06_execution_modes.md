# 6. 执行模式

| 模式 | 入口 | 用途 |
|------|------|------|
| BT | TaskServer + XML | 生产导航 |
| 直驱 | GetPlan / NotifyPath | 调试规划 |
| 多进程 | `autonomy.launch` | 联调 |

```bash
autolink_launch autonomy.launch
# 另开：Bridge 或 Action Client
```

选型：生产用 BT；仅验规划用直驱。
