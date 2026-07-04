# 12. 调度 Scheduler

Scheduler 调度 **CRoutine**（Reader 回调、Component `Proc`、Timer 等）。支持 **classic** 与 **choreography**。

| 本文 §12 | 相关文档 |
|----------|----------|
| 任务调度 | [§0 指南](00_guide.md) · [§10 组件 Component](10_component.md) · [§3 通道 Channel](03_channel.md) · [§1.6 环境变量](01_architecture.md#16-环境与路径变量) |

---

## 12.1 调度流程

`Component::Initialize` 末尾向 Scheduler 注册任务；任务名与 choreography 配置中 `tasks[].name` 对应。

<div class="comm-flow-diagram">
<div class="comm-flow-header">
  <span class="comm-flow-badge">调度</span>
  <span class="comm-flow-title">收包 → CRoutine → Dispatch → 工作线程</span>
  <span class="comm-flow-sub">classic 线程池按 group/prio；choreography 绑核见 [§12.3](#123-classic)–[§12.4](#124-choreography)</span>
</div>

<div class="comm-flow-pipeline comm-flow-pipeline--chain">
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-title">收包</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-title">pending_queue</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-title">CRoutine</span>
    <span class="comm-flow-step-sub">DataVisitor 齐备</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-client">
    <span class="comm-flow-step-title">Dispatch</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-server">
    <span class="comm-flow-step-title">回调 / Proc</span>
  </div>
</div>

<div class="comm-flow-foot">
  <div class="comm-flow-chips">
    <span class="nav-chip">classic</span>
    <span class="nav-chip">choreography</span>
    <span class="nav-chip">AUTOLINK_SCHED_CONF</span>
  </div>
</div>
</div>

---

## 12.2 AUTOLINK_SCHED_CONF

```bash
export AUTOLINK_SCHED_CONF=autolink/autolink/conf/compute_sched_choreography.conf
mainboard -d foo.dag
```

未设置时使用 `autolink.pb.conf` 内默认调度配置。

**配置模板**（`autolink/autolink/conf/`）：

| 文件 | 说明 |
|------|------|
| `example_sched_classic.conf` | classic 教学示例 |
| `example_sched_choreography.conf` | choreography 教学示例 |
| `compute_sched_classic.conf` | 计算管线 |
| `compute_sched_choreography.conf` | 计算管线绑核 |
| `control_sched_*.conf` | 控制管线 |

深度文档：`autolink/docs/source/autolink_scheduler_cn.md`。

---

## 12.3 classic

线程池按 **group** 划分，任务按名称落入 group，组内按 `prio` 优先级调度：

```text
scheduler_conf {
    policy: "classic"
    process_level_cpuset: "0-7,16-23"
    threads: [
        { name: "async_log", cpuset: "1", policy: "SCHED_OTHER", prio: 0 }
        { name: "shm",       cpuset: "2", policy: "SCHED_FIFO", prio: 10 }
    ]
    classic_conf {
        groups: [
            {
                name: "group1"
                processor_num: 16
                affinity: "range"
                cpuset: "0-7,16-23"
                processor_policy: "SCHED_OTHER"
                tasks: [{ name: "E", prio: 0 }]
            },
            {
                name: "group2"
                processor_num: 16
                affinity: "1to1"
                cpuset: "8-15,24-31"
                tasks: [
                    { name: "A", prio: 0 }
                    { name: "B", prio: 1 }
                    { name: "C", prio: 2 }
                    { name: "D", prio: 3 }
                ]
            }
        ]
    }
}
```

| 字段 | 含义 |
|------|------|
| `processor_num` | 组内工作线程数 |
| `affinity` | `range` / `1to1` CPU 亲和 |
| `cpuset` | 可用 CPU 核列表 |
| `tasks[].name` | 与内部任务名匹配 |
| `tasks[].prio` | 组内优先级，数值越大越优先 |

适合通用负载；可为 `async_log`、`shm` 等基础设施线程单独绑核（见 [§8 日志 Log](08_log.md)）。

---

## 12.4 choreography

关键任务指定 **processor 编号**（专用线程），其余任务进入 **pool**：

```text
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
            { name: "A", processor: 0, prio: 1 }
            { name: "B", processor: 0, prio: 2 }
            { name: "C", processor: 1, prio: 1 }
            { name: "E" }
        ]
    }
}
```

未指定 `processor` 的任务（如 `E`）落入 pool。

感知 / 控制等 **延迟敏感链路** 应将任务名绑定到专用 `processor`，减少与日志、通用逻辑的核争抢。

---

## 12.5 联调要点

1. **控制环**： [Channel](03_channel.md) `pending_queue_size=1`，只处理最新帧。  
2. **Proc 非阻塞**：I/O、重计算移出 Scheduler 线程。  
3. **绑核**：control 场景用 `control_sched_choreography.conf`。  
4. **TimerComponent**：`interval` 短于 `Proc` 耗时会导致任务堆积，需缩短 `Proc` 或增大间隔。

---

## 12.6 任务命名

内部任务名通常来自 Component 配置 `config.name` 或 channel 相关工厂。修改 choreography 前，用日志或 `autolink_monitor` 确认实际任务名，再写入 `tasks[].name`。

---

**导航**：[← §11 时间 Time / Rate / Timer](11_timer.md) · [§0 指南](00_guide.md) · [§13 综述 Survey →](13_survey.md)
