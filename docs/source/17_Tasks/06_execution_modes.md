# 6. 执行模式

| 模式 | 入口 | 用途 |
|------|------|------|
| BT + 多进程 | `autonomy.launch` + TaskServer | **默认生产** |
| 直驱规划 | GetPlan / NotifyPath | 调试 planning，不走 Follow |
| Bridge RPC | `NavigationService/Navigate` | 场外客户端 |
```bash
autolink_launch autonomy.launch
# 另终端：grpcurl / rpc-cli / Action Client
```

**什么时候选：** 生产联调 vs 只验规划算法。

### 同步与取消

- BT 模式：`RunWithGoal` 阻塞到终态；Cancel 走 Action 或 Bridge Cancel。  
- Bridge Stream：客户端断连可能触发取消；勿误当成功。  
- `CancelAllGoals`：系统级清槽。

细节：[16 · 引擎](../16_Navigator/03_bt_engine.md)、[15 · Navigate RPC](../15_Bridge/rpcs/07_navigation_command.md)。

### 选型建议

| 需求 | 选 |
|------|-----|
| 完整到点、Recovery | BT |
| 只看 path 是否合理 | 直驱 |
| App / 云 | Bridge + 读 Stream |
