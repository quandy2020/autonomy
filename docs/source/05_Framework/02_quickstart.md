# 2. 快速开始

### 2.1 最小启动流程

```cpp
#include "autolink/autolink.hpp"
#include "autonomy/system/autonomy.hpp"
#include "autonomy/system/options.hpp"

int main(int argc, char** argv) {
    autolink::Init(argv[0]);

    auto options = autonomy::system::CreateOptions(
        "config", "autonomy.lua");

    auto autonomy = autonomy::system::CreateAutonomy(options);
    autonomy->Start();

    autonomy::system::RuntimeOptions runtime;
    runtime.config_directory = "config";
    autonomy->Configure(runtime);

    // 运行循环或调用 NavigateToPose ...
    autonomy->Shutdown();
    autolink::Clear();
    return 0;
}
```

对应可执行逻辑见 `autonomy/system/main.cpp`。

### 2.2 配置前提

确保 `config/autonomy.lua` 存在且包含所需子模块：

```lua
include "map/map.lua"
include "planner/planner.lua"
include "controller/controller.lua"
include "navigator/navigator.lua"

AUTONOMY = {
  map = AUTONOMY_MAP,
  planning = AUTONOMY_PLANNER,
  controller = AUTONOMY_CONTROLLER,
  navigator = navigator,
}
```

### 2.3 命令行参数

通过 `autonomy/common/gflags.hpp`：

| gflag | 说明 |
|-------|------|
| `configuration_directory` | 配置根目录 |
| `configuration_basename` | 主 Lua 文件名（默认 `autonomy.lua`） |
| `--verbose` | 打印版本后退出 |

### 2.4 端到端测试

无需手写 main，使用离线工具：

```bash
./build/bin/autonomy_nav_test --configuration_directory=config
```

详见 [04 Running](../04_Running/04_nav_test.md)。

### 2.5 下一步

| 目标 | 文档 |
|------|------|
| 架构理解 | [§3 框架架构](03_architecture.md) |
| 修改配置 | [§4 配置管线](04_configuration.md) |
| Autolink API | [03 Communication](../03_Communication/00_guide.md) |
