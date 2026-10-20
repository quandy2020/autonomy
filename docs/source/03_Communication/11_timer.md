# 11. 时间 Time / Rate / Timer

示例里用 `Rate` 稳频；车上周期模块用 `TimerComponent` + DAG。各能力对照： 示例里用 `Rate` 稳循环；生产周期任务优先 `TimerComponent` 进 DAG，与 Reader 回调分离。Timer 周期应满足业务带宽（如控制频率），并预留 Scheduler 抖动余量。

| 能力 | 用途 |
|------|------|
| `Time` / `Duration` | 时间戳、间隔 |
| `Rate` | 稳频循环（Binary） |
| `Clock` | 墙钟 / 仿真时钟 |
| `Timer` | 周期回调 |
| `TimerComponent` | 无输入周期 `Proc` |

## Time / Duration / Rate

```cpp
#include "autolink/time/time.hpp"
Time::Now().ToNanosecond();
Duration d(0.1); // 0.1 s
Rate rate(10.0); // 10 Hz
while (autolink::OK()) { /* … */ rate.Sleep(); }
```

消息时间戳常用：`msg->set_timestamp(Time::Now().ToNanosecond())`。

## Clock

| 模式 | 含义 |
|------|------|
| 默认 | 系统墙钟 |
| `MODE_MOCK` | 仿真时钟（测试 / 回放） |

## Timer / TimerComponent

```cpp
// Timer：周期回调
Timer timer(100 /*ms*/, []() { /* … */ }, false);
timer.Start();

// TimerComponent：DAG interval 触发 Proc，无 readers
```

DAG 示例见 [§9](09_launch.md) `timer.dag`。

## 选型

| 场景 | 选择 |
|------|------|
| Binary 稳频发 | `Rate` |
| 进程内周期任务 | `Timer` |
| 生产周期模块 | `TimerComponent` + DAG |
| 仿真时间 | `Clock` MOCK |

Python：`examples/python/py_time.py` · `py_timer.py`。
