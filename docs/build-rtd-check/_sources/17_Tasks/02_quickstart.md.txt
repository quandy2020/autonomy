# 2. 快速开始

### 2.1 使用离线测试工具（推荐）

无需手写代码，用 `autonomy_nav_test` 下发导航任务：

```bash
./build/bin/autonomy_nav_test \
  --configuration_directory=config \
  --start_x=1 --start_y=1 --start_yaw=0 \
  --goal_x=5 --goal_y=5 --goal_yaw=0 \
  --timeout_sec=120
```

详见 [18 Tools · 离线导航测试](../18_Tools/04_nav_test.md)。

### 2.2 C++ API 最小示例

```cpp
#include "autonomy/system/autonomy.hpp"
#include "autonomy/system/options.hpp"

// 初始化后 ...
commsgs::geometry_msgs::PoseStamped goal;
goal.pose.position.x = 5.0;
goal.pose.position.y = 5.0;
goal.header.frame_id = "map";

auto cancel_checker = []() { return false; };
bool ok = autonomy->NavigateToPose(goal, cancel_checker, true, 120.0);
```

### 2.3 配置前提

确保 `config/autonomy.lua` 包含 navigator 配置：

```lua
include "navigator/navigator.lua"

AUTONOMY = {
  -- ...
  navigator = navigator,
}
```

### 2.4 运行时选项

```cpp
autonomy::system::RuntimeOptions runtime;
runtime.config_directory = "config";
runtime.use_bt_navigation = false;  // 当前默认直驱
autonomy->Configure(runtime);
```

### 2.5 下一步

| 目标 | 文档 |
|------|------|
| 任务类型说明 | [§3 任务类型](03_task_types.md) |
| 修改 BT 配置 | [§4 任务配置](04_configuration.md) |
| 理解执行路径 | [§6 执行模式](06_execution_modes.md) |
