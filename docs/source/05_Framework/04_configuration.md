# 4. 配置管线

`*.pb.txt` 怎么变成各 Server 的 `*Options`，以及跨进程必须对齐的字段。

主链是 `autonomy.pb.txt` → `CreateOptions()` → 各 Server 的 `*Options`；改字段要对齐 proto 名与各域 `conf/`。跨进程必须一致的有 frame、map topic、默认 planner/controller id 等。

当前主路径：**Protobuf 文本 conf**（`*.pb.txt`）→ `*Options` → Server。部分旧文档仍提 Lua；以源码 `CreateOptions` 和各域 `conf/` 为准。

## 加载链（共享快照）

```text
autonomy/system/conf/autonomy.pb.txt
        │
system::CreateOptions()
        │
AutonomyOptions
├── planner_options → PlannerServer
├── controller_options → ControllerServer
├── …（perception 等子字段）
```

模块亦可只用本地 `autonomy/<mod>/conf/*.pb.txt`。

## API

```cpp
#include "autonomy/system/options.hpp"
auto options = autonomy::system::CreateOptions("autonomy.pb.txt");
```

## 跨进程须一致

| 参数 | 使用方 |
|------|--------|
| `global_frame` / `robot_base_frame` | map、planner、controller、task |
| `default_planner_id` / `default_controller_id` | planner、controller、task |

## 子模块文档

| 模块 | 文档 |
|------|------|
| Map | [07 Map](../07_Map/00_guide.md) |
| Planning | [08 Planning](../08_Planning/00_guide.md) |
| Control | [09 Control](../09_Control/00_guide.md) |
| Navigator / task | [16 Navigator](../16_Navigator/00_guide.md) |
