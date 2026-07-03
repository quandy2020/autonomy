# 9. Launch 与 mainboard

**mainboard** 加载 DAG、拉起 Component；**autolink_launch** 编排多进程。对标 `ros2 launch`。

| 本文 §9 | 相关文档 |
|---------|----------|
| 进程编排 | [§0 指南](00_guide.md) · [§10 Component](10_component.md) · [§7 Plugin](07_plugin.md) · [§1.6 环境变量](01_architecture.md#16-环境与路径变量) |

Binary 模式（`talker` / `listener`）不经 mainboard；生产部署用 Component + DAG + Launch。

---

## 9.1 启动流程（设计）

`autolink_launch` 解析 `.launch` 后 fork/exec 各 module 的 `mainboard`；`ModuleController::LoadAll()` 完成插件、DAG 与 Component 装配。

<div class="comm-flow-diagram">
<div class="comm-flow-header">
  <span class="comm-flow-badge">启动</span>
  <span class="comm-flow-title">入口 → mainboard → LoadAll → 运行</span>
  <span class="comm-flow-sub">编排层（<code>autolink_launch</code>）与加载层（<code>mainboard</code> / <code>LoadAll</code>）分离；Binary <code>talker</code> 不经此路径</span>
</div>

<div class="comm-flow-fork comm-flow-fork--register">
  <div class="comm-flow-fork-col">
    <span class="comm-flow-fork-label">多进程编排</span>
    <span class="comm-flow-fork-desc"><code>autolink_launch</code> · <code>.launch</code></span>
    <div class="comm-flow-step comm-flow-step-client">
      <span class="comm-flow-step-title">解析 module</span>
      <span class="comm-flow-step-sub">fork / exec mainboard</span>
    </div>
  </div>
  <div class="comm-flow-fork-col">
    <span class="comm-flow-fork-label">单进程直启</span>
    <span class="comm-flow-fork-desc"><code>mainboard -d</code></span>
    <div class="comm-flow-step comm-flow-step-mid">
      <span class="comm-flow-step-title">指定 DAG</span>
      <span class="comm-flow-step-sub">可选 <code>--plugin=</code></span>
    </div>
  </div>
  <div class="comm-flow-fork-join">
    <span class="comm-flow-fork-join-label">mainboard 进程</span>
    <span class="comm-flow-fork-join-arrow" aria-hidden="true">↓</span>
    <div class="comm-flow-step comm-flow-step-server">
      <span class="comm-flow-step-title">LoadAll</span>
      <span class="comm-flow-step-sub"><code>module_controller.cpp</code></span>
    </div>
  </div>
</div>

<div class="comm-flow-pipeline comm-flow-pipeline--chain">
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-title">LoadPlugin</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-title">解析 DAG</span>
    <span class="comm-flow-step-sub"><code>AUTOLINK_DAG_PATH</code></span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-title">ClassLoader</span>
    <span class="comm-flow-step-sub"><code>AUTOLINK_LIB_PATH</code></span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-server">
    <span class="comm-flow-step-title">Init · Proc</span>
    <span class="comm-flow-step-sub">Component / Timer</span>
  </div>
</div>

<table class="comm-flow-legend">
  <tr><th>要点</th><th>说明</th></tr>
  <tr><td>入口</td><td>多 module 用 <code>.launch</code>；单 DAG 可直接 <code>mainboard -d</code></td></tr>
  <tr><td>LoadAll</td><td>插件 → DAG → 加载 <code>.so</code> → <code>Initialize</code> → 调度 <code>Proc</code></td></tr>
  <tr><td>环境变量</td><td><code>AUTOLINK_DAG_PATH</code> · <code>AUTOLINK_LIB_PATH</code> · <code>AUTOLINK_PLUGIN_*</code> · <code>AUTOLINK_SCHED_CONF</code></td></tr>
  <tr><td>与 Binary</td><td><code>talker</code> / <code>listener</code> 用 <code>main()</code>，不经 mainboard（见 §9.6）</td></tr>
</table>
</div>

---

## 9.2 DAG 文件

`common.dag`（`common_component_example`）：

```text
module_config {
    module_library : "autolink/examples/common_component_example/libcommon_component_example.so"
    components {
        class_name : "CommonComponentSample"
        config {
            name : "common"
            readers {
                channel: "/autolink/prediction"
            }
            readers {
                channel: "/autolink/test"
            }
        }
    }
}
```

`timer.dag`（`timer_component_example`）：

```text
module_config {
    module_library : "autolink/examples/timer_component_example/libtimer_component_example.so"
    timer_components {
        class_name : "TimerComponentSample"
        config {
            name : "timer"
            interval : 10
        }
    }
}
```

| 字段 | 含义 |
|------|------|
| `module_library` | Component 动态库（`AUTOLINK_LIB_PATH`） |
| `class_name` | 与 `AUTOLINK_REGISTER_COMPONENT` 注册名一致 |
| `config.name` | Component 实例名 → 内部 Node 名 |
| `readers[].channel` | 输入 channel（多路触发 `Proc`） |
| `interval` | TimerComponent 周期（ms） |

---

## 9.3 Launch 文件

`common.launch`：

```xml
<autolink>
    <module>
        <name>common</name>
        <dag_conf>/autolink/examples/common_component_example/common.dag</dag_conf>
        <process_name>common</process_name>
    </module>
</autolink>
```

`timer.launch` 结构相同，指向 `timer.dag`。

**启动**：

```bash
export AUTOLINK_PATH=<prefix>
mainboard -d autolink/examples/cpp/common_component_example/common.dag

# 或
autolink_launch autolink/examples/cpp/common_component_example/common.launch
```

多 module 的 `.launch` 可在一个文件中声明多个 `<module>`，由 `autolink_launch` 分别起进程。

**mainboard 常用参数**：

```bash
mainboard -d foo.dag
mainboard -d foo.dag --plugin=/path/to/plugins.xml
mainboard -h    # 查看 --plugin、DAG 列表等
```

---

## 9.4 路径与环境变量

环境变量总表见 [§1.6](01_architecture.md#16-环境与路径变量)。Launch / DAG 常用：

| 变量 | 用途 |
|------|------|
| `AUTOLINK_DAG_PATH` | `.dag` 文件 |
| `AUTOLINK_LIB_PATH` | `module_library` `.so` |
| `AUTOLINK_LAUNCH_PATH` | `.launch` 文件 |
| `AUTOLINK_SCHED_CONF` | 调度配置（[§12](12_scheduler.md)） |

DAG / Launch 中路径多为**安装后绝对路径**（如 `/autolink/examples/...`）。开发时需 `cmake --install` 或将构建产物路径写入 DAG。

---

## 9.5 Component 示例运行

```bash
cmake -S autolink -B build/autolink -DAUTOLINK_BUILD_EXAMPLES=ON
cmake --build build/autolink -j8

export AUTOLINK_PATH=build/autolink   # 或安装前缀
export AUTOLINK_LIB_PATH=build/autolink/lib/autolink/examples/...

# 需有 talker 向 /autolink/prediction、/autolink/test 发数据，Proc 才会触发
mainboard -d build/autolink/share/autolink/examples/common_component_example/common.dag
```

Timer 示例可独立运行（无输入 channel）：

```bash
mainboard -d .../timer.dag
# 日志：timer_component_example: Write drivermsg
```

---

## 9.6 Binary vs Launch

| | Binary (`talker`) | mainboard + DAG |
|---|-------------------|-----------------|
| 入口 | `main()` | `Component::Init/Proc` |
| channel 配置 | 硬编码 | DAG 可改无需重编译 |
| 进程模型 | 单 executable | `.so` 动态加载 |
| 适用 | 学习、工具 | 车载 / 多模块部署 |

---

## 9.7 排错

| 现象 | 处理 |
|------|------|
| `no dag conf found` | `AUTOLINK_DAG_PATH`；DAG 路径是否存在 |
| `no module library` | `AUTOLINK_LIB_PATH`；是否编译 examples |
| `Proc` 不执行 | 输入 channel 是否有发布者（Timer 除外） |
| 进程秒退 | 查看 `AERROR`；`Init()` 返回 false |

---

**导航**：[← §8 Log](08_log.md) · [§0 指南](00_guide.md) · [§10 Component →](10_component.md)
