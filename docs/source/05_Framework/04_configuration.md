# 4. 配置管线

当前主路径：**Protobuf 文本 conf**（`*.pb.txt`）→ `*Options` → Server。历史 Lua 管线仍可能出现在部分模块文档，以源码 `CreateOptions` / 各域 `conf/` 为准。

## 4.1 加载链（共享快照）

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

## 4.2 API

```cpp
#include "autonomy/system/options.hpp"
auto options = autonomy::system::CreateOptions("autonomy.pb.txt");
```

## 4.3 跨进程须一致

| 参数 | 使用方 |
|------|--------|
| `global_frame` / `robot_base_frame` | map、planner、controller、task |
| `default_planner_id` / `default_controller_id` | planner、controller、task |

## 4.4 子模块文档

| 模块 | 文档 |
|------|------|
| Map | [07 Map](../07_Map/00_guide.md) |
| Planning | [08 Planning](../08_Planning/00_guide.md) |
| Control | [09 Control](../09_Control/00_guide.md) |
| Navigator / task | [16 Navigator](../16_Navigator/00_guide.md) |
