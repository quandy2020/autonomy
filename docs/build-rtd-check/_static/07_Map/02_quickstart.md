# 2. 快速开始

### 2.1 三步启用 Costmap2D

1. 在 `config/planner/planner.lua` 中配置 `costmap` 块（图层、分辨率、footprint）
2. 在 `config/autonomy.lua` 中启用 `planning = AUTONOMY_PLANNER`（`PlannerServer` 自动构造 `Costmap2DWrapper`）
3. 确保 `MapServer` 发布静态地图，或 `static_layer` 订阅 `/map` 话题

### 2.2 最小 Costmap 配置

```lua
-- config/planner/planner.lua
costmap = {
    enabled = true,
    name = "global_map",
    frame_id = "map",
    resolution = 0.05,
    width = 20.0,
    height = 20.0,
    update_frequency = 5.0,
    robot_radius = 0.22,
    plugins = {"static_layer", "obstacle_layer", "inflation_layer"},

    static_layer = {
        enabled = true,
        map_topic = "map",
    },
    obstacle_layer = {
        enabled = true,
        sensor_sources = {
            scan = { topic = "scan", data_type = "LaserScan", marking = true, clearing = true },
        },
    },
    inflation_layer = {
        enabled = true,
        inflation_radius = 0.35,
        cost_scaling_factor = 3.0,
    },
}
```

### 2.3 C++ 直接使用 Costmap2DWrapper

```cpp
#include "autonomy/map/map_options.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

auto costmap_opts = autonomy::map::CreateCostmap2DOptions("config");
auto costmap = std::make_shared<autonomy::map::costmap_2d::Costmap2DWrapper>(
    costmap_opts, "global_costmap");

costmap->Start();  // 启动后台更新线程

// 注入机器人位姿
costmap->updateRobotPose(x, y, yaw);

// 读取代价值
unsigned char c = costmap->getCostmap()->getCost(mx, my);
```

### 2.4 加载静态地图（MapServer）

```cpp
#include "autonomy/map/map_server.hpp"

auto map_opts = autonomy::map::LoadOptions("config");
auto server = std::make_shared<autonomy::map::MapServer>(map_opts.map_server());

server->SetMapPublishCallback([](const auto& grid) {
    // 将 OccupancyGrid 转发给 Costmap static_layer
});
server->Start();
```

### 2.5 创建 GridMap（2.5D）

```cpp
#include "autonomy/map/grid_map/grid_map_wrapper.hpp"

auto grid_opts = autonomy::map::CreateGridMapOptions("config");
auto grid_map = std::make_shared<autonomy::map::grid_map::GridMapWrapper>(grid_opts);

grid_map->Start();

// 查询高程（需先写入 elevation 层）
grid_map::Position pos(1.0, 2.0);
double elevation;
if (grid_map->getGridMap().atPosition("elevation", pos, elevation)) {
    // 使用高程值
}
```

### 2.6 验证地图是否正常

| 检查项 | 方法 |
|--------|------|
| 静态地图已加载 | `MapServer::HasStaticMap()` 为 true |
| Costmap 在更新 | `costmap->isCurrent()` 为 true |
| 代价值合理 | 自由区 ≈ 0，障碍 ≈ 254，膨胀区 1–252 |
| 坐标变换正确 | `worldToMap` / `mapToWorld` 往返一致 |

更多配置与排错见 [§4 使用指南](04_usage.md)。
