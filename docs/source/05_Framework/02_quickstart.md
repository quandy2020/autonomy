# 2. 快速开始

## 背景

框架层「怎么起 Server、共享 conf 从哪来」的最短说明。完整运行命令在 [04 Running](../04_Running/02_quickstart.md)。

## 思路

按固定顺序执行：依赖/环境 → 构建或配置 → 启动 → 最小验证；分支放到文末链接。


## 步骤

### 1. 多进程启动

```bash
source scripts/setup_environment.bash
autolink_launch autonomy.launch
```

各 `*_main.cpp`：`CreateOptions`（或模块 conf）→ Server → `WaitForShutdown`。

### 2. 共享配置

```cpp
#include "autonomy/system/options.hpp"
auto options = autonomy::system::CreateOptions("autonomy.pb.txt");
// PlannerServer(options.planner_options()) …
```

路径：`autonomy/system/conf/autonomy.pb.txt` · `autonomy/<mod>/conf/`。

### 3. 发令

`CreateAutonomy` / `autonomy_nav_test` 已移除。用 Bridge 或 Action Client。
