# 8. 调度与传输

本章说明 autolink 的 **Scheduler + CRoutine** 调度子系统，以及 **Transport** 多后端传输机制。

## 8.1 调度系统概览

```
Task 创建
    │
    ▼
Scheduler::Dispatch(task)
    │
    ├─ classic: 按优先级入 group 队列
    └─ choreography: 按配置处理器 ID 入队
    │
    ▼
Processor 线程池
    │
    ▼
CRoutine::Resume() → 用户回调 / Component::Proc
```

**设计动机**：自动驾驶场景需要可预测的延迟与 CPU 亲和性，通用 OS 线程调度不足以满足硬实时链路需求。

## 8.2 CRoutine（协程）

- 用户态协程，切换开销远小于 pthread
- 每个 Reader 回调、Component `Proc` 可作为独立 Task 调度
- 与 `Processor` 线程绑定，避免跨核迁移（choreography 模式）

## 8.3 Classic 策略

**适用**：不熟悉全车 DAG 拓扑时的**通用**策略。

### 8.3.1 配置结构

```text
scheduler_conf {
    policy: "classic"
    process_level_cpuset: "0-7,16-23"
    classic_conf {
        groups: [
            {
                name: "group1"
                processor_num: 16
                affinity: "range"          # 或 "1to1"
                cpuset: "0-7,16-23"
                processor_policy: "SCHED_OTHER"
                processor_prio: 0
                tasks: [
                    { name: "task_name", prio: 2 }
                ]
            }
        ]
    }
}
```

### 8.3.2 关键配置项

| 字段 | 说明 |
|------|------|
| `groups` | 资源隔离；可按 NUMA 划分 |
| `process_level_cpuset` | mainboard 进程可用 CPU |
| `affinity` | `range`：线程可在 cpuset 内浮动；`1to1`：一线程一核 |
| `processor_policy` | `SCHED_FIFO` / `SCHED_RR` / `SCHED_OTHER` |
| `processor_prio` | 实时策略 1–99；OTHER 为 nice -20–19 |
| `tasks[].prio` | 同 group 内多优先级队列，数值大优先 |

### 8.3.3 优先级反转

拓扑中越靠后的任务应配置**更高** `prio`，避免上游阻塞下游（详见 `autolink/docs/source/autolink_scheduler_cn.md` 图示）。

## 8.4 Choreography 策略

**适用**：对任务依赖、耗时、CPU 占用**充分掌握**后的精细编排。

```protobuf
scheduler_conf {
    policy: "choreography"
    choreography_conf {
        choreography_processor_num: 8
        choreography_cpuset: "0-7"
        choreography_processor_policy: "SCHED_FIFO"
        choreography_processor_prio: 10

        pool_processor_num: 8
        pool_cpuset: "16-23"
        pool_processor_policy: "SCHED_OTHER"

        tasks: [
            { name: "lidar_proc", processor: 0, prio: 10 }
            { name: "fusion",     processor: 2, prio: 8  }
        ]
    }
}
```

| 区域 | 用途 |
|------|------|
| **choreography** 处理器 | 关键链路任务绑死到指定 CPU |
| **pool** 处理器 | 非关键 / 默认可抢占任务 |

配置文件示例：`autolink/autolink/conf/compute_sched_choreography.conf`

## 8.5 传输层架构

```
Transport (Singleton)
├── IntraDispatcher    # 同进程
├── ShmDispatcher      # 共享内存
│   ├── Segment (POSIX / XSI)
│   ├── Block / Notifier
│   └── ProtobufArenaManager
└── RtpsParticipant    # FastDDS（可选）
```

### 8.5.1 创建接口

```cpp
Transport::Instance()->CreateTransmitter<M>(attr, OptionalMode::SHM);
Transport::Instance()->CreateReceiver<M>(attr, listener, OptionalMode::SHM);
```

### 8.5.2 模式对比

| 模式 | 延迟 | 吞吐 | 跨进程 | 跨机 |
|------|------|------|--------|------|
| INTRA | 最低 | 最高 | ❌ | ❌ |
| SHM | 低 | 高 | ✅ 同机 | ❌ |
| RTPS | 中 | 中 | ✅ | ✅ |

### 8.5.3 共享内存路径

1. `ShmTransmitter` 将序列化数据写入 `Segment`
2. `ConditionNotifier` / `MulticastNotifier` 唤醒订阅方
3. `ShmReceiver` 读取并反序列化，触发 listener

大块 protobuf 可配合 `ProtobufArenaManager` 减少分配。

## 8.6 QoS 与传输协同

`qos_profile_conf` 定义默认 depth、可靠性。高频传感器建议：

- `depth` ≥ 10
- SHM 模式
- choreography 绑核避免与规划争抢

控制环路建议：

- `depth` = 1
- 实时调度策略 `SCHED_FIFO`

## 8.7 服务发现与传输

发现层（UDP）与数据层（SHM/RTPS）分离：

1. Writer 注册 → `ChannelManager` 广播
2. Reader 注册 → 匹配 Writer，建立 Transmitter-Receiver 对
3. 数据传输不经过发现通道

## 8.8 Record 子系统（简要）

| 组件 | 职责 |
|------|------|
| `RecordWriter` | 按 channel 写 Section 到文件 |
| `RecordReader` | 顺序读消息 |
| `RecordViewer` | 按时间索引跳转 |
| `Player` | 回放注入 Writer，支持速率控制 |

文件格式：Header + 多个 Section（channel meta / message chunk）。

## 8.9 mainboard 与调度

```
mainboard -d foo.dag
    │
    ├─ 解析 DAG，ClassLoader 加载 .so
    ├─ 初始化 Scheduler（读 AUTOLINK_SCHED_CONF 或默认）
    ├─ 为每个 Component 创建 DataVisitor + Task
    └─ 阻塞直至信号退出
```

环境变量 `AUTOLINK_PATH` 指向配置与 lib 目录。

## 8.10 调优检查清单

| 检查项 | 建议 |
|--------|------|
| 关键任务是否绑核 | choreography + 1to1 |
| 日志线程是否独立 | `threads { name: "async_log" }` |
| SHM 是否跨 NUMA | group 按 NUMA 拆分 |
| 队列深度是否合理 | 控制 1，传感 10+ |
| 回放是否用仿真时钟 | `Clock::SetMode(MOCK)` |

## 8.11 参考配置路径

| 文件 | 说明 |
|------|------|
| `autolink/autolink/conf/example_classic_sched.conf` | classic 示例 |
| `autolink/autolink/conf/example_choreography_sched.conf` | choreography 示例 |
| `autolink/autolink/conf/compute_sched_choreography.conf` | 计算图编排 |
| `autolink/autolink/conf/control_sched_choreography.conf` | 控制链路编排 |

更完整的调度说明见子模块文档 `autolink/docs/source/autolink_scheduler_cn.md`。
