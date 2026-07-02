(communication-overview)=
# 1. 模块概览

### 1.1 定位

| 维度 | 说明 |
|------|------|
| 层级 | Autonomy **通信中间件**（Middleware / Runtime） |
| 职责 | 进程间消息发布订阅、RPC 服务、长时间任务 Action、参数服务、录制回放 |
| 消息载体 | Protocol Buffers（`google::protobuf::Message`）及 POD 类型 |
| 上游依赖 | `protobuf`、`FastDDS`（可选 RTPS）、共享内存子系统 |
| 下游消费 | `autonomy/*` 各算法模块、`mainboard` 组件加载、`bridge` 外部桥接 |
| 对标 | Apollo Cyber RT、`rclcpp` + DDS、ROS 2 `rmw` |

### 1.2 核心能力

| 能力 | 状态 | 说明 |
|------|------|------|
| Node / Writer / Reader | ✅ | 发布订阅，Channel 拓扑自动发现 |
| Service / Client | ✅ | 同步 / 异步 RPC |
| Action Server / Client | ✅ | Goal / Feedback / Result 长任务 |
| Parameter Server / Client | ✅ | 全局参数读写 |
| Component 组件模型 | ✅ | DAG 驱动、多输入融合 `Proc()` |
| 调度器（classic / choreography） | ✅ | CRoutine 协程 + 多优先级队列 |
| 传输（INTRA / SHM / RTPS） | ✅ | 按 `OptionalMode` 与配置选择 |
| Record / Play | ✅ | `autolink_recorder` 工具链 |
| Python API | ✅ | `py_autolink` 绑定 |
| Timer / Rate / Time | ✅ | 与 ROS `rclcpp::Time` 语义对齐 |
| Plugin / ClassLoader | ✅ | 动态库组件加载 |

### 1.3 源码结构

```
autolink/
├── autolink/
│   ├── autolink.hpp / init.cpp        # 框架入口、CreateNode
│   ├── node/                          # Node、Reader、Writer
│   ├── service/                       # Service、Client
│   ├── action/                        # Action Server / Client
│   ├── transport/                     # 传输层（INTRA/SHM/RTPS）
│   ├── service_discovery/             # 拓扑发现（Channel/Node/Service）
│   ├── scheduler/                     # classic / choreography 调度
│   ├── croutine/                      # 用户态协程
│   ├── component/                     # Component 基类与注册宏
│   ├── data/                          # DataVisitor、ChannelBuffer
│   ├── parameter/                     # 参数服务
│   ├── record/                        # 录制读写
│   ├── message/                       # Protobuf 工厂与 traits
│   ├── timer/ / time/                 # 定时与仿真时钟
│   ├── mainboard/                     # 进程入口、DAG 加载
│   ├── tools/                         # monitor、recorder、channel 工具
│   ├── python/                        # Python 绑定
│   ├── conf/                          # 调度、传输配置
│   └── proto/                         # 框架内部 proto 定义
├── examples/                          # C++ / Python 示例
└── docs/source/                       # 子模块文档（可参考）
```

### 1.4 在 Autonomy 栈中的位置

```
┌─────────────────────────────────────────────────────────┐
│  Application: planning / control / localization / map │
└───────────────────────────┬─────────────────────────────┘
                            │ C++ commsgs struct
                            ▼
┌─────────────────────────────────────────────────────────┐
│  commsgs: ToProto / FromProto                           │
└───────────────────────────┬─────────────────────────────┘
                            │ protobuf Message
                            ▼
┌─────────────────────────────────────────────────────────┐
│  autolink: Node → Writer/Reader/Service/Action          │
└───────────────────────────┬─────────────────────────────┘
                            │ INTRA / SHM / RTPS
                            ▼
┌─────────────────────────────────────────────────────────┐
│  OS: 进程间 / 网络                                       │
└─────────────────────────────────────────────────────────┘
```

### 1.5 相关模块

| 模块 | 关系 |
|------|------|
| [commsgs](../14_Commsgs/01_overview.md) | 消息类型定义；算法层 struct，边界 ToProto |
| [Framework](../05_Framework/index.rst) | Autonomy 系统启动与模块装配 |
| [Bridge](../15_Bridge/01_overview.md) | gRPC / MQTT 与外部系统桥接 |
| 各算法包 | 通过 `CreateNode` 创建 pub/sub 与 action |

### 1.6 与 ROS 2 / Cyber RT 对照

| 概念 | ROS 2 | Cyber RT | Autolink |
|------|-------|----------|----------|
| 基本单元 | Node | Node | `autolink::Node` |
| 发布订阅 | Publisher / Subscription | Writer / Reader | `CreateWriter` / `CreateReader` |
| 话题名 | topic | channel | channel |
| RPC | Service / Client | Service / Client | `CreateService` / `CreateClient` |
| 长任务 | Action | — | `action::CreateServer` / `CreateClient` |
| 组件 | — | Component | `Component<M...>` + DAG |
| 调度 | Executor | Scheduler + CRoutine | `classic` / `choreography` |
| 发现 | DDS discovery | RTPS / UDP | `service_discovery` |
| 录制 | rosbag2 | cyber_recorder | `autolink_recorder` |
