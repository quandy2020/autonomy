(communication-architecture)=
# 1. 架构 Architecture

栈内位置、内部分层、部署形态、数据流与生命周期。上手见 [§0](00_guide.md)。

## 1.1 栈内位置

算法模块经 **Node + 消息边界** 与 autolink 交互，不直接操作 DDS/SHM。

```text
应用层（planning · control · localization · task）
│ ToProto / FromProto（commsgs struct）
通信 API（Node：Writer/Reader · Service · Action · Parameter）
│ INTRA · SHM · RTPS（可选）
传输层（同进程 / 同机 / 跨主机可选）
```

| 相关 | 关系 |
|------|------|
| [commsgs](../14_Commsgs/index.rst) | 消息类型与 proto |
| [Framework](../05_Framework/index.rst) | Server 装配 |
| [Bridge](../15_Bridge/index.rst) | 外部协议 |

## 1.2 内部分层

发现与数据路径分离：拓扑匹配后，业务数据不经发现通道。

| 层 | 职责 |
|----|------|
| L1 应用 | `CreateWriter/Reader` · Service/Client · Action · Parameter |
| L2 发现 | 本机文件总线（Autolink 拓扑）；DDS 端点发现不属本层 |
| L3 传输 | 按拓扑选 INTRA / SHM / RTPS（可选） |
| L4 调度 | Scheduler + CRoutine（回调 / `Proc` / Timer） |
| L5 扩展 | Component `.so` · Plugin · DAG |

## 1.3 部署形态

| 形态 | 入口 | 调度 | 适用 |
|------|------|------|------|
| Binary | `main()` → `Init` + `CreateNode` | 自建 `Rate` | 示例、工具 |
| Component | `mainboard` / `autolink_launch` | Scheduler + DataVisitor | 生产、车载 |

Binary 流程：`Init` → `CreateNode` → Writer/Reader → `Rate::Sleep` → `WaitForShutdown`。 
Component 流程：`LoadLibrary` → `Init` → DataVisitor → `Proc`（DAG `readers[]`）。

见 [§2](02_node.md) · [§9](09_launch.md) · [§10](10_component.md)。

## 1.4 Component 数据流

```text
DAG readers[] → Reader → DataVisitor（多路齐备）→ Scheduler → Proc() → Writer
```

`TimerComponent` 无 `readers`，由 DAG `interval`（ms）触发 `Proc()`，见 [§11](11_timer.md)。直驱回调见 [§3.7](03_channel.md)。

## 1.5 生命周期

| 状态 | 含义 |
|------|------|
| Init | `autolink::Init(argv)`，加载配置 |
| Ready | `CreateNode` 或 `mainboard LoadAll` |
| Running | `OK()` 循环；回调 / `Proc` / Timer |
| Shutdown | SIGINT；`Component::Shutdown` → `Clear` |

| 阶段 | Binary | Component |
|------|--------|-----------|
| 启动 | `Init` → `CreateNode` | `mainboard -d foo.dag` |
| 运行 | `while (OK())` + `Rate` | Scheduler 调度 `Proc` |
| 退出 | 析构 / `Clear` | `Shutdown` → `UnloadLibrary` |

## 1.6 环境变量

| 变量 | 用途 |
|------|------|
| `AUTOLINK_PATH` | 运行时根（须含 `conf/autolink.pb.conf`） |
| `AUTOLINK_DISTRIBUTION_HOME` | 安装前缀 |
| `AUTOLINK_CONF_PATH` | 模块 / 调度 conf |
| `AUTOLINK_DAG_PATH` | `.dag` |
| `AUTOLINK_LIB_PATH` | Component `.so` |
| `AUTOLINK_LAUNCH_PATH` | `.launch` |
| `AUTOLINK_PLUGIN_*` | 插件描述与 `.so` |
| `AUTOLINK_SCHED_CONF` | 调度配置 |
| `AUTOLINK_DOMAIN_ID` / `AUTOLINK_IP` | 可选跨机 RTPS（需 FastDDS） |
| `GLOG_*` | 日志，见 [§8](08_log.md) |

## 1.7 CLI

| 工具 | 作用 |
|------|------|
| `mainboard` | `-d foo.dag`；`--plugin=` |
| `autolink_launch` | 多进程 `.launch` |
| `autolink_channel` / `autolink_monitor` | 诊断 / 流量 |
| `autolink_recorder` | 录放 |

构建：`-DAUTOLINK_BUILD_TOOLS=ON`。

## 1.8 Autonomy 集成

各 Server 进程内 `CreateNode`，commsgs ↔ autolink：

| Server | 用法 |
|--------|------|
| Planning | Plugin 加载 `GlobalPlanner` |
| Task | Action Client → Controller |
| 各模块 | conf + 可选 Parameter |
