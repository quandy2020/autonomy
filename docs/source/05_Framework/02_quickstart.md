# 2. 快速开始

### 2.1 多进程启动

```bash
source scripts/setup_environment.bash
autolink_launch autonomy.launch
```

各 `*_main.cpp`：`CreateOptions`（或模块 conf）→ Server → `WaitForShutdown`。细节见 [04 Running](../04_Running/02_quickstart.md)。

### 2.2 共享配置

```cpp
#include "autonomy/system/options.hpp"
auto options = autonomy::system::CreateOptions("autonomy.pb.txt");
// PlannerServer(options.planner_options()) …
```

路径：`autonomy/system/conf/autonomy.pb.txt` · `autonomy/<mod>/conf/`。

### 2.3 发令

`CreateAutonomy` / `autonomy_nav_test` 已移除。用 Bridge 或 Action Client。
