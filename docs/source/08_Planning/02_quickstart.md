# 2. 快速开始

### 2.1 三步启用

1. 编辑 `config/planner/planner.lua`
2. 在 `config/autonomy.lua` 中设置 `planning = AUTONOMY_PLANNER`
3. 启动后 `system::Autonomy` 自动构造 `PlannerServer`

### 2.2 最小示例

```cpp
#include "autolink/common/init.hpp"
#include "autonomy/planning/planner_server.hpp"

autolink::Init(argv[0]);
auto options = autonomy::planning::CreateOptions("config");
auto server = std::make_shared<autonomy::planning::PlannerServer>(options);

commsgs::geometry_msgs::PoseStamped start, goal;
// 填充 start / goal ...

auto path = server->GetPlan(
    start, goal, "navfn_planner",
    []() { return false; });  // cancel_checker
```

### 2.3 切换规划器

```lua
-- config/planner/planner.lua
default_planner_id = "navfn_planner"  -- 或 dijkstra_planner / theta_star_planner
```
