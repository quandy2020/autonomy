# 12. 调度 Scheduler

Reader 回调、Component `Proc`、Timer 由 Scheduler 调度。控制环延迟敏感时可换 **choreography** 绑核。

默认 scheduler 对多数模块够用；控制环 latency 敏感时在 `autolink.pb.conf` 切 choreography 并绑核。Reader 与 Timer 抢同一核会导致 cmd 周期不稳，先 profiling 再绑。

## 配置

```bash
export AUTOLINK_SCHED_CONF=autolink/autolink/conf/compute_sched_choreography.conf
mainboard -d foo.dag
```

未设置时用 `autolink.pb.conf` 默认。模板目录：`autolink/autolink/conf/`（`example_sched_*`、`compute_sched_*`、`control_sched_*`）。

## classic

线程池按 group；任务按名入组，组内按 `prio` 调度。

| 字段 | 含义 |
|------|------|
| `processor_num` | 组内工作线程数 |
| `affinity` | `range` / `1to1` |
| `cpuset` | CPU 核列表 |
| `tasks[].name` / `prio` | 任务名与优先级 |

适合通用负载；可为 `async_log`、`shm` 单独绑核。

## choreography

关键任务绑 **processor 编号**；其余进 pool。适合控制环等延迟敏感链路。

## 联调

| 要点 | 说明 |
|------|------|
| 任务名 | 与 conf 中 `tasks[].name` 匹配 |
| 绑核 | 确认 cpuset 不与热路径冲突 |
| 深度文档 | `autolink/docs/source/autolink_scheduler_cn.md` |
