# 7. Plugin

Plugin 按**类名**从动态库或进程内注册表实例化派生类型。对标 ROS `pluginlib`。

| 本文 §7 | 相关文档 |
|---------|----------|
| 可插拔算法 | [§0 指南](00_guide.md) · [§9 Launch](09_launch.md) · [§10 Component](10_component.md) |

与 [Component](10_component.md) 的区别：Component 由 DAG 加载；Plugin 由业务 `CreateInstance` 加载算法实现。

---

## 7.1 设计

Plugin 按**类名**从动态库或进程内注册表实例化派生类型。与 [Component](10_component.md) 的区别：Component 由 DAG 加载；Plugin 由业务 `CreateInstance` 加载算法实现。

<div class="comm-flow-diagram">
<div class="comm-flow-header">
  <span class="comm-flow-badge">插件</span>
  <span class="comm-flow-title">描述文件 / 进程内注册 → CreateInstance</span>
  <span class="comm-flow-sub"><code>mainboard</code> 可 <code>--plugin=</code> 或自动 <code>LoadInstalledPlugins()</code></span>
</div>

<div class="comm-flow-fork">
  <div class="comm-flow-fork-col">
    <span class="comm-flow-fork-label">外部插件</span>
    <span class="comm-flow-fork-desc">XML 描述文件 + <code>class_loader</code> 动态库</span>
    <div class="comm-flow-fork-track comm-flow-fork-track--dense">
      <div class="comm-flow-fork-row">
        <div class="comm-flow-step comm-flow-step-client">
          <span class="comm-flow-step-label">描述</span>
          <span class="comm-flow-step-title">plugin.xml</span>
        </div>
        <span class="comm-flow-link-arrow" aria-hidden="true">→</span>
        <div class="comm-flow-step comm-flow-step-mid">
          <span class="comm-flow-step-label">解析</span>
          <span class="comm-flow-step-title">PluginDescription</span>
          <span class="comm-flow-step-sub">ParseFromDescriptionFile</span>
        </div>
      </div>
      <div class="comm-flow-fork-row">
        <div class="comm-flow-step comm-flow-step-mid">
          <span class="comm-flow-step-label">加载</span>
          <span class="comm-flow-step-title">LoadPlugin</span>
          <span class="comm-flow-step-sub">PluginManager</span>
        </div>
        <span class="comm-flow-link-arrow" aria-hidden="true">→</span>
        <div class="comm-flow-step comm-flow-step-mid">
          <span class="comm-flow-step-label">动态库</span>
          <span class="comm-flow-step-title">.so</span>
          <span class="comm-flow-step-sub">class_loader 加载</span>
        </div>
      </div>
    </div>
  </div>
  <div class="comm-flow-fork-col">
    <span class="comm-flow-fork-label">进程内注册</span>
    <span class="comm-flow-fork-desc">主程序已静态链接，无需拆 <code>.so</code></span>
    <div class="comm-flow-fork-track comm-flow-fork-track--single">
      <div class="comm-flow-step comm-flow-step-client">
        <span class="comm-flow-step-label">注册</span>
        <span class="comm-flow-step-title">RegisterInProcessClass</span>
        <span class="comm-flow-step-sub">已链接 · 无需 .so</span>
      </div>
    </div>
  </div>
  <div class="comm-flow-fork-join">
    <span class="comm-flow-fork-join-label">两路径汇合 · 按类名实例化</span>
    <span class="comm-flow-fork-join-arrow" aria-hidden="true">↓</span>
    <div class="comm-flow-step comm-flow-step-server">
      <span class="comm-flow-step-label">实例化</span>
      <span class="comm-flow-step-title">CreateInstance</span>
      <span class="comm-flow-step-sub"><code>CreateInstance&lt;Base&gt;</code></span>
      <span class="comm-flow-step-sub">(class_name)</span>
    </div>
  </div>
</div>

<div class="comm-flow-foot">
  <div class="comm-flow-chips">
    <span class="nav-chip">mainboard --plugin=</span>
    <span class="nav-chip">LoadInstalledPlugins()</span>
    <span class="nav-chip">AUTOLINK_PLUGIN_*</span>
  </div>
</div>

<table class="comm-flow-legend">
  <tr><th>路径</th><th>适用</th></tr>
  <tr><td>外部插件</td><td>XML 描述 + 动态库，规划器等可热插拔算法</td></tr>
  <tr><td>进程内注册</td><td>主程序已静态链接实现，无需拆 <code>.so</code></td></tr>
</table>
</div>

---

## 7.2 描述文件格式

XML 由 `PluginDescription::ParseFromDescriptionFile` 解析（`plugin_description.cpp`）：

```xml
<library path="lib/libmy_planner.so">
  <class type="MyPlanner" base_class="autonomy::planning::common::GlobalPlanner"/>
  <class type="AnotherPlanner" base_class="autonomy::planning::common::GlobalPlanner"/>
</library>
```

| 元素 | 含义 |
|------|------|
| `library@path` | 动态库路径（相对 `AUTOLINK_PLUGIN_LIB_PATH`） |
| `class@type` | 派生类名（`CreateInstance` 参数） |
| `class@base_class` | 基类 RTTI 名 |

环境变量见 [§1.6](01_architecture.md#16-环境与路径变量)（`AUTOLINK_PLUGIN_*`）。

---

## 7.3 PluginManager API

```cpp
#include "autolink/plugin_manager/plugin_manager.hpp"

auto* pm = autolink::plugin_manager::PluginManager::Instance();

// 加载描述文件
pm->LoadPlugin("/path/to/planner_plugins.xml");

// 进程内注册（无需 XML）
pm->RegisterInProcessClass<common::GlobalPlanner>("NavfnPlanner");

// 按类名创建
auto planner = pm->CreateInstance<common::GlobalPlanner>("NavfnPlanner");

// 列举某基类的所有派生类名
auto names = pm->GetDerivedClassNameByBaseClass<common::GlobalPlanner>();
```

`RegisterInProcessClass` 适用于主程序已静态链接实现、不想拆 `.so` 的场景。

---

## 7.4 Autonomy 集成示例

`autonomy/planning/planner_server.cpp` 在启动时注册内置规划器并加载外部描述文件：

```cpp
void LoadExternalPlugins(const proto::PlannerOptions& options) {
    auto* pm = PluginPm::Instance();
    pm->RegisterInProcessClass<common::GlobalPlanner>("NavfnPlanner");
    pm->RegisterInProcessClass<common::GlobalPlanner>("DijkstraPlanner");
    pm->RegisterInProcessClass<common::GlobalPlanner>("ThetaStarPlanner");
    for (const auto& path : options.planner_plugin_libraries()) {
        if (!pm->LoadPlugin(path)) {
            AWARN << "Failed to load plugin description: " << path;
        }
    }
    pm->LoadInstalledPlugins();
}

common::GlobalPlanner::SharedPtr CreatePlannerInstance(const std::string& type, ...) {
    const std::string resolved = ResolvePlannerClass(type);
    if (resolved == "NavfnPlanner") {
        return std::make_shared<planner::navfn::NavfnPlanner>(...);
    }
    // 外部插件
    auto instance = PluginPm::Instance()->CreateInstance<common::GlobalPlanner>(resolved);
    return common::GlobalPlanner::SharedPtr(std::move(instance));
}
```

配置中通过 `planner_plugin_libraries` 传入 XML 路径列表。

---

## 7.5 与 Component / class_loader

| | Component | Plugin |
|---|-----------|--------|
| 注册 | `AUTOLINK_REGISTER_COMPONENT` | XML `<class>` 或 `RegisterInProcessClass` |
| 加载触发 | DAG `class_name` | `LoadPlugin` / `CreateInstance` |
| 生命周期 | `Init` / `Proc` / `Shutdown` | 由业务持有 `shared_ptr` |
| 典型用途 | 通信驱动模块 | 算法策略替换 |

Component 的 `.so` 同样经 `class_loader` 加载；单元测试见 `autolink/autolink/class_loader/class_loader_test.cpp`（`libplugin1.so` / `libplugin2.so`）。

---

## 7.6 Launch / mainboard 集成

```bash
mainboard -d foo.dag --plugin=/path/to/plugins.xml
```

`.launch` XML：

```xml
<autolink>
    <module>
        <name>planning</name>
        <dag_conf>...</dag_conf>
        <plugin>path/to/planner_plugins.xml</plugin>
    </module>
</autolink>
```

`autolink_launch` 解析 `<plugin>` 列表并传给 `mainboard`（`autolink/tools/autolink_launch/main.cpp`）。

---

## 7.7 排错

| 现象 | 处理 |
|------|------|
| `CreateInstance` 返回 null | 类名拼写；`LoadPlugin` 是否成功；`base_class` 是否匹配 |
| 找不到 `.so` | 检查 `AUTOLINK_PLUGIN_LIB_PATH` |
| 重复加载 | 同一描述文件勿多次 `LoadPlugin`（有去重警告） |

---

**导航**：[← §6 Parameter](06_parameter.md) · [§0 指南](00_guide.md) · [§8 Log →](08_log.md)
