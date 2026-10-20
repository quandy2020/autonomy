# 7. API 参考

**什么时候查：** 写嵌入代码或对照进程名；**日常发令**更常用 launch + Bridge/Action。

### 进程入口

| 入口 | 说明 |
|------|------|
| `autolink_launch autonomy.launch` | 多进程栈 |
| `autonomy.task` | TaskServer 二进制 |
| Bridge `autonomy.bridge` | 需 `BUILD_GRPC=ON` |

### C++（TaskServer）

```cpp
auto server = std::make_shared<autonomy::task::TaskServer>();
server->Configure(TaskServer::DefaultOptions());
server->Start();
```

Conf：`CreateOptions("navigator.pb.txt")` 等在 `autonomy/task/navigation/options.hpp`；系统级 `system::CreateOptions("autonomy.pb.txt")`。

### 状态枚举

`NavigatorState`：Idle / Running / Completed / Failed / Canceled / Shutdown（`interface.hpp`）。

### 对外 RPC / Action

- gRPC：`automsgs.rpcs.*` → [15 rpcs](../15_Bridge/rpcs/02_service_overview.md)  
- Action：与 `automsgs` nav action 定义一致  

不要 invent `CreateAutonomy` 或单进程 BtNavigator 入口。
