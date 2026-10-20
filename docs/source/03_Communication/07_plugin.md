# 7. 插件 Plugin

按类名从动态库或进程内注册表实例化派生类型。类似 `pluginlib`。

## 7.1 加载流程

```text
LoadPlugin(xml) / RegisterInProcessClass → CreateInstance<Base>(type) → shared_ptr
```

## 7.2 plugin.xml

```xml
<library path="lib/libmy_planner.so">
<class type="MyPlanner" base_class="autonomy::planning::common::GlobalPlanner"/>
</library>
```

| 元素 | 含义 |
|------|------|
| `library@path` | 相对 `AUTOLINK_PLUGIN_LIB_PATH` |
| `class@type` | `CreateInstance` 参数 |
| `class@base_class` | 基类 RTTI 名 |

## 7.3 API

```cpp
auto* pm = autolink::plugin_manager::PluginManager::Instance();
pm->LoadPlugin("/path/to/planner_plugins.xml");
pm->RegisterInProcessClass<common::GlobalPlanner>("NavfnPlanner");
auto planner = pm->CreateInstance<common::GlobalPlanner>("NavfnPlanner");
```

## 7.4 与 Component

| — | Component | Plugin |
|---|-----------|--------|
| 注册 | `AUTOLINK_REGISTER_COMPONENT` | XML 或 `RegisterInProcessClass` |
| 触发 | DAG `class_name` | `CreateInstance` |
| 用途 | 通信驱动模块 | 算法策略替换 |

```bash
mainboard -d foo.dag --plugin=/path/to/plugins.xml
```

Autonomy 集成见 `planner_server.cpp`（内置注册 + `planner_plugin_libraries`）。
