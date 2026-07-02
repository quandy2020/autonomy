# 5. Autolink 模块架构设计

本文描述 `autolink` 的逻辑分层、核心子系统关系与运行时数据流。

## 5.1 设计目标

autolink 遵循以下设计原则：

1. **去中心化发现**：无 master，节点对等，通过 UDP 拓扑同步
2. **多传输后端**：INTRA / SHM / RTPS 对用户透明可配
3. **组件化部署**：Component + DAG + mainboard，与 Cyber RT 一致
4. **协程调度**：CRoutine 降低线程开销，支持 classic / choreography 策略
5. **Protobuf 生态**：消息序列化、配置、录制统一 proto 格式
6. **ROS 语义对齐**：Node / topic(channel) / service / action 概念对应，降低迁移成本

## 5.2 实现状态

| 子系统 | 实现度 | 说明 |
|--------|--------|------|
| Node / Writer / Reader | ✅ | 完整 API |
| Service / Client | ✅ | 同步 + 异步 |
| Action | ✅ | Server / Client 状态机 |
| Service Discovery | ✅ | Channel / Node / Service 管理器 |
| Transport INTRA | ✅ | 进程内指针传递 |
| Transport SHM | ✅ | 共享内存段 + notifier |
| Transport RTPS | ✅ | FastDDS 集成 |
| Scheduler classic | ✅ | 多 group 多优先级 |
| Scheduler choreography | ✅ | 绑核编排 |
| Component | ✅ | 多输入 `Proc` + 注册宏 |
| Record / Play | ✅ | 文件分段存储 |
| Parameter | ✅ | 全局参数服务 |
| Python 绑定 | ✅ | 主要 API 覆盖 |
| PluginManager | ✅ | 动态库加载 |

## 5.3 分层架构

<div class="plan-arch-diagram">

  <div class="plan-arch-layer plan-arch-app">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">应用层</span>
      <span class="plan-arch-title">Autonomy 算法模块 / 用户节点</span>
      <span class="plan-arch-sub">planning · control · localization · 自定义 Component</span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-body-block">
        <div class="nav-body-label">对外接口</div>
        <div class="nav-chip-list">
          <span class="nav-chip">CreateNode</span>
          <span class="nav-chip">CreateWriter&lt;T&gt;</span>
          <span class="nav-chip">CreateReader&lt;T&gt;</span>
          <span class="nav-chip">CreateService</span>
          <span class="nav-chip">action::CreateServer</span>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>API 调用</span></div>

  <div class="plan-arch-layer plan-arch-server">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">节点层</span>
      <span class="plan-arch-title">Node</span>
      <span class="plan-arch-sub">NodeChannelImpl · NodeServiceImpl</span>
    </div>
    <div class="plan-arch-body plan-arch-body-cols">
      <div class="nav-body-block">
        <div class="nav-body-label">Channel 路径</div>
        <ul>
          <li>Writer → Transmitter → Dispatcher</li>
          <li>Receiver → 回调 / DataVisitor</li>
        </ul>
      </div>
      <div class="nav-body-block">
        <div class="nav-body-label">Service 路径</div>
        <ul>
          <li>Service 注册回调处理请求</li>
          <li>Client 发送并等待响应</li>
        </ul>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>注册 / 匹配</span></div>

  <div class="plan-arch-split">
    <div class="plan-arch-layer plan-arch-plugin">
      <div class="plan-arch-header">
        <span class="plan-arch-badge">中间件层</span>
        <span class="plan-arch-title">Service Discovery</span>
      </div>
      <div class="plan-arch-body">
        <div class="nav-body-block">
          <div class="nav-body-label">管理器</div>
          <div class="nav-chip-list">
            <span class="nav-chip">ChannelManager</span>
            <span class="nav-chip">NodeManager</span>
            <span class="nav-chip">ServiceManager</span>
          </div>
        </div>
        <div class="nav-body-block">
          <div class="nav-body-label">存储</div>
          <div class="nav-chip-list">
            <span class="nav-chip">MultiValueWarehouse</span>
            <span class="nav-chip">Graph</span>
          </div>
        </div>
      </div>
    </div>

    <div class="plan-arch-link">
      <span class="plan-arch-link-text">驱动</span>
      <span class="plan-arch-link-arrow">↔</span>
    </div>

    <div class="plan-arch-layer plan-arch-map">
      <div class="plan-arch-header">
        <span class="plan-arch-badge">传输层</span>
        <span class="plan-arch-title">Transport</span>
      </div>
      <div class="plan-arch-body">
        <div class="nav-body-block">
          <div class="nav-body-label">后端</div>
          <div class="nav-chip-list">
            <span class="nav-chip">IntraDispatcher</span>
            <span class="nav-chip">ShmDispatcher</span>
            <span class="nav-chip">RtpsParticipant</span>
          </div>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>调度执行</span></div>

  <div class="plan-arch-split">
    <div class="plan-arch-layer plan-arch-post">
      <div class="plan-arch-header">
        <span class="plan-arch-badge">调度层</span>
        <span class="plan-arch-title">Scheduler + CRoutine</span>
      </div>
      <div class="plan-arch-body">
        <div class="nav-body-block">
          <div class="nav-body-label">策略</div>
          <div class="nav-chip-list">
            <span class="nav-chip">classic</span>
            <span class="nav-chip">choreography</span>
          </div>
        </div>
      </div>
    </div>

    <div class="plan-arch-link">
      <span class="plan-arch-link-text">支撑</span>
      <span class="plan-arch-link-arrow">↔</span>
    </div>

    <div class="plan-arch-layer plan-arch-post">
      <div class="plan-arch-header">
        <span class="plan-arch-badge">运行时</span>
        <span class="plan-arch-title">mainboard / Component</span>
      </div>
      <div class="plan-arch-body">
        <div class="nav-body-block">
          <div class="nav-body-label">加载</div>
          <div class="nav-chip-list">
            <span class="nav-chip">DAG</span>
            <span class="nav-chip">ClassLoader</span>
            <span class="nav-chip">PluginManager</span>
          </div>
        </div>
      </div>
    </div>
  </div>

</div>

## 5.4 消息发布数据流

```
用户 Write(msg)
    │
    ▼
Writer<MessageT>
    │ 序列化（ProtobufTraits）
    ▼
Transmitter<M>  ──mode──▶ INTRA | SHM | RTPS
    │
    ▼
Dispatcher 路由至所有匹配 Receiver
    │
    ▼
Receiver<M> 回调 / 入队 ChannelBuffer
    │
    ▼
Reader 用户回调 或 Component::Proc
    │
    ▼
Scheduler 调度 CRoutine 执行
```

## 5.5 Component 执行模型

```
DAG 描述 readers ──▶ DataVisitor 聚合多路输入
                          │
                          ▼
                   全部输入就绪？
                          │ yes
                          ▼
              创建 Task → 入队 Scheduler
                          │
                          ▼
                   CRoutine::Resume
                          │
                          ▼
                   Component::Proc(M0, M1, ...)
```

- `Component<M0,M1,...>` 模板参数指定输入消息类型
- `AUTOLINK_REGISTER_COMPONENT` 导出工厂符号供 mainboard 加载
- `TimerComponent` 定时触发，无输入 channel

## 5.6 目录与职责映射

| 目录 | 职责 |
|------|------|
| `node/` | 对外 API 门面，Writer/Reader 实现 |
| `transport/` | Transmitter/Receiver/Dispatcher/SHM 段 |
| `service_discovery/` | 拓扑注册、变更通知、图查询 |
| `scheduler/` | Processor 线程池、任务队列、绑核 |
| `croutine/` | 协程上下文切换 |
| `data/` | DataVisitor、ChannelBuffer、通知 |
| `component/` | 组件基类、初始化流程 |
| `action/` | Action 状态机封装 |
| `record/` | 分段文件格式、读写器 |
| `mainboard/` | 进程入口、ModuleController |
| `message/` | Protobuf 工厂、类型注册 |

## 5.7 与 Autonomy 系统集成

```
config/autonomy.lua
        │
        ▼
system::Autonomy 构造各 Server
        │
        ├─▶ 各 Server 内部 CreateNode("xxx_server")
        ├─▶ CreateReader / CreateWriter 对接 commsgs
        └─▶ Action Server 供 Navigator 行为树调用
```

- 算法 Server（如 `ControllerServer`）**持有** autolink Node，不暴露给应用层
- 消息类型来自 `autonomy/commsgs`，边界 `ToProto`
- 详见 [Framework](../05_Framework/03_architecture.md)

## 5.8 扩展点

| 扩展点 | 机制 |
|--------|------|
| 新算法模块 | 继承 `Component` 或独立 `main` + Node |
| 新消息类型 | commsgs proto + traits，Writer/Reader 实例化 |
| 新传输 | 实现 `Transmitter`/`Receiver` 并注册 Dispatcher |
| 调度策略 | 新增 `Scheduler` 子类 + factory |
| 外部协议 | 通过 [Bridge](../15_Bridge/01_overview.md) 网关转换 |
