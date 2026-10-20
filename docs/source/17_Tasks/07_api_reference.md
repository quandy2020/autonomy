# 7. API 参考

| 入口 | 说明 |
|------|------|
| `autolink_launch autonomy.launch` | 多进程栈 |
| `autonomy.task` | TaskServer |
| Bridge / Action Client | 发令 |

```cpp
auto server = std::make_shared<autonomy::task::TaskServer>();
server->Configure(TaskServer::DefaultOptions());
server->Start();
```

共享 conf：`system::CreateOptions("autonomy.pb.txt")`；Navigator：`task::navigation::CreateOptions("navigator.pb.txt")`。

`NavigatorState`：Idle / Running / Completed / Failed / Canceled / Shutdown（`interface.hpp`）。
