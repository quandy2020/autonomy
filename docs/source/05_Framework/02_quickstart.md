# 2. 快速开始

### 2.1 推荐：多进程启动

```bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
export AUTONOMY_BT_PLUGIN_PATH="$PWD/build/lib"
export GLOG_logtostderr=1

autolink_launch autonomy.launch
```

各 `*_main.cpp` 内自行 `CreateOptions`（或模块 conf）→ 构造对应 Server → `autolink::WaitForShutdown`。详见 [04 Running](../04_Running/02_quickstart.md)。

### 2.2 共享配置快照

planning / control / perception 等可加载同一 `AutonomyOptions` 文本，只取子字段：

```cpp
#include "autonomy/system/options.hpp"

auto options = autonomy::system::CreateOptions("autonomy.pb.txt");
// 例如：PlannerServer(options.planner_options()) …
```

路径约定见 `autonomy/system/conf/autonomy.pb.txt` 与各模块 `autonomy/<mod>/conf/`。

### 2.3 命令行参数

通过 `autonomy/common/gflags.hpp` 与各进程 `--conf=`：

| 参数 / gflag | 说明 |
|--------------|------|
| `--conf=` | 模块 / 共享 conf 文本（如 `autonomy.pb.txt`） |
| `configuration_directory` 等 | 历史 gflag，视模块而定 |
| `--verbose` | 打印版本后退出（若支持） |

### 2.4 端到端验证

进程内 `CreateAutonomy` / `autonomy_nav_test` **已移除**。发令用 Bridge 或 autolink Action Client。

### 2.5 下一步

| 目标 | 文档 |
|------|------|
| 架构理解 | [§3 框架架构](03_architecture.md) |
| 修改配置 | [§4 配置管线](04_configuration.md) |
| Autolink API | [03 Communication](../03_Communication/00_guide.md) |
