(communication-architecture)=
# 1. 模块架构

> 上手与环境见 [§0](00_guide.md)；各模块 API 见 §2–§12。本文描述 **栈内位置、内部分层、部署形态、数据流与生命周期**。

| 本文 §1 | 相关文档 |
|---------|----------|
| 分层、数据流、环境 | [§0 指南](00_guide.md) · [§13 综述](13_survey.md) |

<div class="nav-costmap-banner">
  <strong>autolink 在 Autonomy 栈中的位置</strong>
  <span class="nav-costmap-detail">算法 struct → commsgs ToProto → Node/Writer/Reader → 跨进程</span>
  <span class="nav-costmap-arrow">Service · Action · Parameter · Plugin 同栈扩展 →</span>
</div>

---

## 1.1 栈内位置

Autonomy 算法模块不直接操作 DDS/SHM，而是经 **Node + 消息边界** 与 `autolink` 交互。

<div class="plan-arch-diagram">

  <div class="plan-arch-layer plan-arch-app">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">应用层</span>
      <span class="plan-arch-title">planning · control · localization · navigator</span>
      <span class="plan-arch-sub">C++ <code>autonomy::commsgs::*</code> struct，进程内算法逻辑</span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-body-block">
        <div class="nav-body-label">典型 Server</div>
        <div class="nav-chip-list">
          <span class="nav-chip">PlannerServer</span>
          <span class="nav-chip">ControllerServer</span>
          <span class="nav-chip">BtNavigator</span>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>ToProto / FromProto</span></div>

  <div class="plan-arch-layer plan-arch-server">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">通信 API</span>
      <span class="plan-arch-title">autolink Node</span>
      <span class="plan-arch-sub">Writer/Reader · Service/Client · Action · Parameter</span>
    </div>
    <div class="plan-arch-body plan-arch-body-cols">
      <div class="nav-body-block">
        <div class="nav-body-label">持续数据流</div>
        <div class="nav-chip-list">
          <span class="nav-chip">Channel</span>
          <span class="nav-chip">/plan</span>
          <span class="nav-chip">/cmd_vel</span>
        </div>
      </div>
      <div class="nav-body-block">
        <div class="nav-body-label">事务 / 长任务</div>
        <div class="nav-chip-list">
          <span class="nav-chip">Service</span>
          <span class="nav-chip">Action</span>
          <span class="nav-chip">Parameter</span>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>INTRA · SHM · RTPS</span></div>

  <div class="plan-arch-layer plan-arch-map">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">传输层</span>
      <span class="plan-arch-title">同进程 / 同机 / 跨主机</span>
      <span class="plan-arch-sub">发现：UDP 多播，无 central master</span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-chip-list">
        <span class="nav-chip">INTRA</span>
        <span class="nav-chip">SHM</span>
        <span class="nav-chip">RTPS</span>
        <span class="nav-chip">Bridge → ROS 2</span>
      </div>
    </div>
  </div>

</div>

| 相关模块 | 关系 |
|----------|------|
| [commsgs](../14_Commsgs/index.rst) | 消息类型与 proto |
| [Framework](../05_Framework/index.rst) | Server 装配与 Lua 配置 |
| [Bridge](../15_Bridge/index.rst) | 外部协议互通 |

---

## 1.2 内部分层

`autolink` 运行时自顶向下分为五层；**发现与数据路径分离**——拓扑匹配后，业务数据不经发现通道。

<div class="plan-arch-diagram">

  <div class="plan-arch-layer plan-arch-app">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">L1 应用</span>
      <span class="plan-arch-title">Node 端点</span>
      <span class="plan-arch-sub">CreateWriter/Reader · Service/Client · Action · Parameter</span>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>RoleAttributes · channel 名 · QoS</span></div>

  <div class="plan-arch-layer plan-arch-server">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">L2 发现</span>
      <span class="plan-arch-title">Service Discovery</span>
      <span class="plan-arch-sub">UDP 多播 · 拓扑注册与匹配</span>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>匹配完成</span></div>

  <div class="plan-arch-layer plan-arch-map">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">L3 传输</span>
      <span class="plan-arch-title">Transport</span>
      <span class="plan-arch-sub">按拓扑自动选择 INTRA / SHM / RTPS</span>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>收包 → pending_queue</span></div>

  <div class="plan-arch-layer plan-arch-plugin">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">L4 调度</span>
      <span class="plan-arch-title">Scheduler + CRoutine</span>
      <span class="plan-arch-sub">Reader 回调 · Component::Proc · Timer（classic / choreography）</span>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>class_loader / PluginManager</span></div>

  <div class="plan-arch-layer plan-arch-post">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">L5 扩展</span>
      <span class="plan-arch-title">Component .so · 算法 Plugin</span>
      <span class="plan-arch-sub">DAG 配置 · XML 描述文件 · RegisterInProcessClass</span>
    </div>
  </div>

</div>

---

## 1.3 两种部署形态

| 形态 | 入口 | 调度 | 适用 |
|------|------|------|------|
| **Binary** | `main()` → `Init` + `CreateNode` | 自建 `Rate` 循环 | 示例、工具、快速验证 |
| **Component** | `mainboard` / `autolink_launch` | Scheduler + DataVisitor | 生产模块、车载部署 |

<div class="plan-arch-diagram">
  <div class="plan-arch-split">

    <div class="plan-arch-layer plan-arch-app">
      <div class="plan-arch-header">
        <span class="plan-arch-badge">Binary</span>
        <span class="plan-arch-title">talker / listener</span>
      </div>
      <div class="plan-arch-body">
        <ul>
          <li><code>Init</code> → <code>CreateNode</code></li>
          <li><code>CreateWriter</code> / <code>CreateReader</code></li>
          <li><code>Rate::Sleep</code> 主循环</li>
          <li><code>WaitForShutdown</code></li>
        </ul>
      </div>
    </div>

    <div class="plan-arch-link">
      <span class="plan-arch-link-text">对比</span>
      <span class="plan-arch-link-arrow">⇄</span>
    </div>

    <div class="plan-arch-layer plan-arch-server">
      <div class="plan-arch-header">
        <span class="plan-arch-badge">Component</span>
        <span class="plan-arch-title">DAG + mainboard</span>
      </div>
      <div class="plan-arch-body">
        <ul>
          <li><code>LoadLibrary</code> → <code>Init</code></li>
          <li>DataVisitor 对齐多路输入</li>
          <li>Scheduler 驱动 <code>Proc</code></li>
          <li>channel 名由 DAG 配置</li>
        </ul>
      </div>
    </div>

  </div>
</div>

详见 [§2 Node](02_node.md) · [§9 Launch](09_launch.md) · [§10 Component](10_component.md)。

---

## 1.4 Component 数据流

多输入 Component 由 **DataVisitor** 缓存并对齐各路最新消息，齐备后唤醒 `Proc`。

<div class="comm-flow-diagram">
<div class="comm-flow-header">
  <span class="comm-flow-badge">数据流</span>
  <span class="comm-flow-title">DAG → 收包 → 对齐 → 调度 → 输出</span>
  <span class="comm-flow-sub">channel 名由 DAG <code>readers[]</code> 注入；Writer 在 <code>Init()</code> 中创建</span>
</div>

<div class="comm-flow-pipeline comm-flow-pipeline--chain">
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-label">①</span>
    <span class="comm-flow-step-title">DAG</span>
    <span class="comm-flow-step-sub">readers[]</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-label">②</span>
    <span class="comm-flow-step-title">Reader</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-label">③</span>
    <span class="comm-flow-step-title">DataVisitor</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-label">④</span>
    <span class="comm-flow-step-title">Scheduler</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-client">
    <span class="comm-flow-step-label">⑤</span>
    <span class="comm-flow-step-title">Proc()</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-server">
    <span class="comm-flow-step-label">⑥</span>
    <span class="comm-flow-step-title">Writer</span>
  </div>
</div>

<div class="comm-flow-foot">
  <div class="comm-flow-chips">
    <span class="nav-chip">多路齐备才触发</span>
    <span class="nav-chip">TimerComponent 无 readers</span>
    <span class="nav-chip">见 §3.7 直驱回调</span>
  </div>
</div>
</div>

`TimerComponent` 无输入 channel，由 DAG `interval`（ms）直接触发 `Proc()`，见 [§11](11_timer.md)。

Channel 直驱回调路径见 [§3.7](03_channel.md#37-数据路径设计)。

---

## 1.5 生命周期

<div class="comm-flow-diagram">
<div class="comm-flow-header">
  <span class="comm-flow-badge">状态机</span>
  <span class="comm-flow-title">Init → Ready → Running → Shutdown</span>
  <span class="comm-flow-sub">Binary 与 Component 共用 <code>autolink::Init</code>；运行期调度方式不同</span>
</div>

<div class="comm-flow-pipeline comm-flow-pipeline--chain">
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-label">状态</span>
    <span class="comm-flow-step-title">Init</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-title">Ready</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-client">
    <span class="comm-flow-step-title">Running</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-title">Shutdown</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-server">
    <span class="comm-flow-step-title">Clear</span>
  </div>
</div>

<table class="comm-flow-legend">
  <tr><th>状态</th><th>触发 / 含义</th></tr>
  <tr><td>Init</td><td><code>autolink::Init(argv)</code>，加载配置与子系统</td></tr>
  <tr><td>Ready</td><td><code>CreateNode</code> 或 <code>mainboard LoadAll</code></td></tr>
  <tr><td>Running</td><td><code>OK()</code> 循环；Reader 回调 / <code>Proc</code> / Timer</td></tr>
  <tr><td>Shutdown</td><td>SIGINT 或异常；<code>Component::Shutdown</code></td></tr>
</table>
<table class="comm-flow-legend">
  <tr><th>阶段</th><th>Binary</th><th>Component</th></tr>
  <tr><td>启动</td><td><code>Init</code> → <code>CreateNode</code></td><td><code>mainboard -d foo.dag</code></td></tr>
  <tr><td>运行</td><td><code>while (OK())</code> + <code>Rate</code></td><td>Scheduler 调度 <code>Proc</code></td></tr>
  <tr><td>退出</td><td>析构 / <code>Clear</code></td><td><code>Shutdown</code> → <code>UnloadLibrary</code></td></tr>
</table>
</div>

---

## 1.6 环境与路径变量

`GetFilePathWithEnv` 按变量解析相对路径；未设置时回退到 `AUTOLINK_PATH` 下默认目录。

| 变量 | 解析对象 | 典型值 |
|------|----------|--------|
| `AUTOLINK_PATH` | 运行时根（`conf/autolink.pb.conf`） | `build/autolink` 或 `/usr/local/share/autolink` |
| `AUTOLINK_DISTRIBUTION_HOME` | 安装前缀 | `/usr/local` |
| `AUTOLINK_CONF_PATH` | 模块配置、调度 conf | `$AUTOLINK_PATH/conf` |
| `AUTOLINK_DAG_PATH` | `.dag` 文件 | 安装 share 目录 |
| `AUTOLINK_LIB_PATH` | Component `.so` | `lib/autolink/examples/...` |
| `AUTOLINK_LAUNCH_PATH` | `.launch` 文件 | 同 DAG |
| `AUTOLINK_PLUGIN_*` | 插件描述与 `.so` | 见 [§7](07_plugin.md) |
| `AUTOLINK_SCHED_CONF` | 调度配置 | `conf/compute_sched_choreography.conf` |
| `AUTOLINK_DOMAIN_ID` | RTPS 发现域 | `setup.bash` 默认 `80` |
| `GLOG_*` | 日志 | 见 [§8](08_log.md) |

Python 另需 `PYTHONPATH` 指向 `autolink_py3`（`examples/python/README.md`）。

---

## 1.7 CLI 工具

| 工具 | 作用 |
|------|------|
| `mainboard` | `-d foo.dag` 加载 Component；`--plugin=` 加载插件 |
| `autolink_launch` | 多 module 进程编排 |
| `autolink_channel` | 诊断 channel 匹配 |
| `autolink_monitor` | 实时流量 |
| `autolink_recorder` | 录放（[§3.9](03_channel.md#39-调试工具)） |

构建：`cmake ... -DAUTOLINK_BUILD_TOOLS=ON`。

---

## 1.8 Autonomy 集成

`system::Autonomy` 各 Server 在内部 `CreateNode`，将 commsgs 与 autolink 对接：

| Server | autolink 用法 |
|--------|---------------|
| Planning | [Plugin](07_plugin.md) 加载 `GlobalPlanner` |
| Navigator | [Action](05_action.md) Client → Controller |
| 各模块 | Lua 配置 + 可选 [Parameter](06_parameter.md) |

详见 [Framework §3](../05_Framework/03_architecture.md)。

---

**导航**：[← §0 指南](00_guide.md) · [§2 Node →](02_node.md)
