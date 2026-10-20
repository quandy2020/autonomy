# 7. 插件 Plugin

按类名从 `.so` 或进程内注册表实例化算法，类似 `pluginlib`。Autonomy 规划/控制插件在 [Framework §6](../05_Framework/06_plugin_system.md) 里接 Server。 Autolink 插件按「类名 + 库路径」实例化；Autonomy 里 Planner/Controller/BT 节点在 Framework 的 PluginManager 与 Server 上再接一层。新增算法先实现接口并登记 loader，否则 conf 里的 id 无法构造。

## 加载流程

```text
LoadPlugin(xml) / RegisterInProcessClass → CreateInstance<Base>(type) → shared_ptr
```

## plugin.xml

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

## API

```cpp
auto* pm = autolink::plugin_manager::PluginManager::Instance();
pm->LoadPlugin("/path/to/planner_plugins.xml");
pm->RegisterInProcessClass<common::GlobalPlanner>("NavfnPlanner");
auto planner = pm->CreateInstance<common::GlobalPlanner>("NavfnPlanner");
```

## 与 Component

| 维度 | Component | Plugin |
|------|-----------|--------|
| 注册 | `AUTOLINK_REGISTER_COMPONENT` | XML 或 `RegisterInProcessClass` |
| 触发 | DAG `class_name` | `CreateInstance` |
| 用途 | 通信驱动模块 | 算法策略替换 |

```bash
mainboard -d foo.dag --plugin=/path/to/plugins.xml
```

Autonomy 集成见 `planner_server.cpp`（内置注册 + `planner_plugin_libraries`）。
