# 7. API 参考

### 7.1 运行入口（推荐）

| 入口 | 说明 |
|------|------|
| `autolink_launch autonomy.launch` | 拉起 task / planning / control 等 |
| `autonomy.task` | `TaskServer` 进程 |
| Bridge / Action Client | 外部发令 |

进程内 `CreateAutonomy` / `system::Autonomy` **已移除**。

### 7.2 TaskServer

头文件：`autonomy/task/task_server.hpp`

```cpp
auto server = std::make_shared<autonomy::task::TaskServer>();
auto options = autonomy::task::TaskServer::DefaultOptions();
server->Configure(options);
server->Start();
// … WaitForShutdown …
server->Shutdown();
```

导航 / 建图 / 遥操等目标提交接口以 `TaskServer` 与各 `*Task` 实现为准。

### 7.3 共享 conf（CreateOptions）

规划 / 控制等进程可加载共享快照：

```cpp
#include "autonomy/system/options.hpp"

auto autonomy_opts = autonomy::system::CreateOptions("autonomy.pb.txt");
// 取 planning / controller 等子字段构造对应 Server
```

Navigator 本地 conf：`autonomy::task::navigation::CreateOptions("navigator.pb.txt")`。

### 7.4 NavigatorInterface

```cpp
enum class NavigatorState {
    kIdle, kRunning, kCompleted, kFailed, kCanceled, kShutdown
};
```

见 `autonomy/task/navigation/interface.hpp`。

### 7.5 相关文档

- [05 Framework · 快速开始](../05_Framework/02_quickstart.md)
- [14 Commsgs · nav_msgs](../14_Commsgs/08_nav_planning_msgs.md)
- [04 Running](../04_Running/03_autonomy_process.md)
