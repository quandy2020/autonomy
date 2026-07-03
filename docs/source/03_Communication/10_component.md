# 10. Component

Component 是推荐的**算法模块形态**：`.so` + DAG，支持 **1–4 路**输入（`Component<M0,…,M3>`）。

| 本文 §10 | 相关文档 |
|----------|----------|
| 算法模块 | [§0 指南](00_guide.md) · [§9 Launch](09_launch.md) · [§3 Channel](03_channel.md) · [§12 Scheduler](12_scheduler.md) |

---

## 10.1 设计

Component 将算法封装为 `.so`，由 DAG 声明输入 channel；框架在数据齐备时调度 `Proc()`。完整多路数据流见 [§1.4](01_architecture.md#14-component-数据流)。

<div class="comm-flow-diagram">
<div class="comm-flow-header">
  <span class="comm-flow-badge">Component</span>
  <span class="comm-flow-title">DAG → Initialize → Init / Proc → Writer</span>
  <span class="comm-flow-sub"><code>AUTOLINK_REGISTER_COMPONENT</code> 供 mainboard 按 <code>class_name</code> 实例化</span>
</div>

<div class="comm-flow-pipeline comm-flow-pipeline--chain">
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-title">DAG</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-title">Initialize</span>
    <span class="comm-flow-step-sub">框架注册 Reader</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-title">DataVisitor</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-client">
    <span class="comm-flow-step-title">Proc()</span>
    <span class="comm-flow-step-sub">用户 Init() 在 Initialize 内</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-server">
    <span class="comm-flow-step-title">Writer</span>
  </div>
</div>

<table class="comm-flow-legend">
  <tr><th>钩子</th><th>调用方</th></tr>
  <tr><td><code>Initialize(config)</code></td><td>框架：创建 Node、注入 Reader、注册调度</td></tr>
  <tr><td><code>Init()</code> / <code>Proc()</code></td><td>用户：一次性初始化 / 每帧业务逻辑</td></tr>
</table>
</div>

---

## 10.2 双输入 Component 示例

**头文件**（`common_component_example.hpp`）：

```cpp
#include "autolink/class_loader/class_loader.hpp"
#include "autolink/component/component.hpp"
#include "examples.pb.h"

class CommonComponentSample : public Component<Driver, Driver> {
public:
    bool Init() override;
    bool Proc(const std::shared_ptr<Driver>& msg0,
              const std::shared_ptr<Driver>& msg1) override;
};
AUTOLINK_REGISTER_COMPONENT(CommonComponentSample)
```

**实现**（`common_component_example.cpp`）：

```cpp
bool CommonComponentSample::Init() {
    AINFO << "Commontest component init";
    return true;
}

bool CommonComponentSample::Proc(const std::shared_ptr<Driver>& msg0,
                                 const std::shared_ptr<Driver>& msg1) {
    AINFO << "Start common component Proc [" << msg0->msg_id() << "] ["
          << msg1->msg_id() << "]";
    return true;
}
```

**DAG**（`common.dag`）— 两个 `readers` 对应 `msg0`、`msg1`：

```text
components {
    class_name : "CommonComponentSample"
    config {
        name : "common"
        readers { channel: "/autolink/prediction" }
        readers { channel: "/autolink/test" }
    }
}
```

仅当 **两路 channel 均有新消息** 且对齐后，`Proc` 才被调用（同步多传感器融合的典型模式）。

---

## 10.3 单路与多路模板

| 基类 | Proc 签名 |
|------|-----------|
| `Component<M0>` | `Proc(const shared_ptr<M0>&)` |
| `Component<M0,M1>` | `Proc(..., ...)` 两参 |
| 至多 | `Component<M0,M1,M2,M3>` 四路 |

`component.hpp` 注释：勿手动调用 `Init`/`Proc`，由框架调度。

---

## 10.4 输出 Channel

在 `Init()` 中通过 `node_` 创建 Writer（`TimerComponent` 示例同理）：

```cpp
bool TimerComponentSample::Init() {
    driver_writer_ = node_->CreateWriter<Driver>("/carstatus/channel");
    return true;
}
```

`Proc` 内只 `Write`，勿重复 `CreateWriter`。

---

## 10.5 编译与加载

`common_component_example/CMakeLists.txt` 将示例编译为 `libcommon_component_example.so`，安装到 `AUTOLINK_LIB_PATH`。mainboard 流程：

```cpp
class_loader_manager_.LoadLibrary(load_path);
auto component = class_loader_manager_.CreateClassObj<ComponentBase>(class_name);
component->Initialize(conf);
```

见 `module_controller.cpp`。

---

## 10.6 启动与联调

```bash
export AUTOLINK_PATH=...
export AUTOLINK_LIB_PATH=.../libcommon_component_example.so 所在目录

mainboard -d .../common.dag
```

`Proc` 需要两路输入：用两个 talker 或改为单路 `Component<Driver>` 测试。TimerComponent 无此依赖，见 [§11 Timer 与 Time](11_timer.md)。

---

## 10.7 与 Binary 对比

| | Binary | Component |
|---|--------|-----------|
| Node 创建 | `CreateNode` in `main` | 框架注入 `node_` |
| 触发方式 | 自建循环 / Reader 回调 | DataVisitor 同步多路 |
| 配置变更 | 重编译 | 改 DAG |
| 部署 | 单 binary | `.so` + mainboard |

---

## 10.8 排错

| 现象 | 处理 |
|------|------|
| `Proc` 从未调用 | 某路 channel 无数据；检查 DAG `readers` 名 |
| `Initialize` 失败 | `Init()` 返回 false；Writer 创建失败 |
| 库加载失败 | `class_name` 与 `AUTOLINK_REGISTER_COMPONENT` 不一致 |
| 延迟高 | `Proc` 内阻塞；迁移到异步或缩简逻辑 |

---

**导航**：[← §9 Launch](09_launch.md) · [§0 指南](00_guide.md) · [§11 Timer 与 Time →](11_timer.md)
