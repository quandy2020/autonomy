# 4. 配置管线

Autonomy 采用 **Lua → Protobuf → Server** 统一配置管线，与 Cartographer / Autoware 风格一致。

### 4.1 加载链

```
config/autonomy.lua
        │
        ▼
ConfigurationFileResolver
        │
        ▼
LuaParameterDictionary
        │
        ▼
system::LoadOptions()  →  proto::AutonomyOptions
        │
        ├── map_options        → MapServer
        ├── planner_options    → PlannerServer
        ├── controller_options → ControllerServer
        ├── navigator_options  → navigator / BT
        └── transform_options  → TransformServer
```

### 4.2 API

```cpp
#include "autonomy/system/options.hpp"

auto options = autonomy::system::CreateOptions(
    "config",           // configuration_directory
    "autonomy.lua");    // configuration_basename
```

`system/options.cpp` 按 key 分发：

| Lua key | Loader | Proto 字段 |
|---------|--------|------------|
| `map` | `map::LoadOptions` | `map_options` |
| `planning` | `planning::LoadOptions` | `planner_options` |
| `controller` | `control::LoadOptions` | `controller_options` |
| `navigator` | `navigator::LoadOptions` | `navigator_options` |
| `transform` | `transform::LoadOptions` | `transform_options` |

### 4.3 顶层 `autonomy.lua`

```lua
include "common.lua"
include "map/map.lua"
include "planner/planner.lua"
include "controller/controller.lua"
include "navigator/navigator.lua"
include "transform/transform.lua"

AUTONOMY = {
  map = AUTONOMY_MAP,
  planning = AUTONOMY_PLANNER,
  controller = AUTONOMY_CONTROLLER,
  navigator = navigator,
  transform = AUTONOMY_TRANSFORM,
}
return AUTONOMY
```

### 4.4 共享参数 `common.lua`

跨模块参数须在多处保持一致：

| 参数 | 使用方 |
|------|--------|
| `global_frame` | map, planner, controller, navigator |
| `robot_base_frame` | 同上 |
| `default_planner_id` | planner, navigator |
| `default_controller_id` | controller, navigator |
| `goal_reached_tolerance` | controller, navigator |

### 4.5 运行时覆盖

`RuntimeOptions` 可在 `Configure()` 时覆盖配置：

```cpp
autonomy::system::RuntimeOptions runtime;
runtime.planner_id = "theta_star_planner";
runtime.use_bt_navigation = true;
runtime.global_frame = "map";
autonomy->Configure(runtime);
```

### 4.6 子模块配置文档

| 模块 | 配置文件 | 文档 |
|------|----------|------|
| Map | `config/map/map.lua` | [07 Map](../07_Map/04_usage.md) |
| Planning | `config/planner/planner.lua` | [08 Planning](../08_Planning/00_guide.md) |
| Control | `config/controller/controller.lua` | [09 Control](../09_Control/00_guide.md) |
| Navigator | `config/navigator/navigator.lua` | [16 Navigator](../16_Navigator/04_usage.md) |

### 4.7 相关文档

- [§2 快速开始](02_quickstart.md)
- [02 Installation · 环境配置](../02_Installation/07_environment.md)
