# 11. Timer 与 Time

时间相关能力：`Time` / `Duration` / `Rate` / `Clock` 用于时间戳与稳频；`Timer` / `TimerComponent` 用于周期任务。

| 本文 §11 | 相关文档 |
|----------|----------|
| 时间与定时 | [§0 指南](00_guide.md) · [§10 Component](10_component.md) |

| 模块 | 头文件 | 用途 |
|------|--------|------|
| Time / Duration | `time/time.hpp` · `duration.hpp` | 时间戳、间隔、睡眠 |
| Rate | `time/rate.hpp` | 主循环稳频 |
| Clock | `time/clock.hpp` | 系统 / Mock 时钟 |
| Timer | `timer/timer.hpp` | 时间轮周期回调 |
| TimerComponent | `component/timer_component.hpp` | DAG 周期 `Proc` |

---

## 11.1 Time 与 Duration（C++）

### Time — 时间点

内部以 **纳秒**（`uint64_t`）存储，对标 ROS `rclcpp::Time`。

```cpp
#include "autolink/time/time.hpp"

using autolink::Time;
using autolink::Duration;

// 当前墙钟时间（经 Clock 单例，默认同系统时间）
Time now = Time::Now();
uint64_t ns = now.ToNanosecond();
double sec = now.ToSecond();

// 单调时钟（不受 NTP 调整影响，适合测间隔）
Time mono = Time::MonoTime();

// 构造：纳秒 / 秒 / 秒+纳秒
Time t1(1234567890ULL);
Time t2(1.5);                    // 1.5 秒
Time t3(2, 500000000);           // 2s + 500ms

// 睡眠至指定时刻
Time::SleepUntil(now + Duration(1.0));
```

**消息时间戳**（`talker.cpp`、`service.cpp` 等）：

```cpp
msg->set_timestamp(Time::Now().ToNanosecond());
```

### Duration — 时间间隔

```cpp
Duration d1(100000000);          // 100ms（纳秒）
Duration d2(0.5);                // 0.5 秒
Duration d3(1, 0);               // 1 秒

d2.Sleep();                      // 阻塞当前线程 0.5s

Time t = Time::Now();
Time t2 = t + d2;
Time t3 = t - d1;
Duration elapsed = t2 - t;       // 两 Time 相减得 Duration
```

| API | 说明 |
|-----|------|
| `ToNanosecond()` / `ToSecond()` | 单位转换 |
| `IsZero()` | 是否为 0 |
| `Sleep()` | 阻塞睡眠 |
| `operator+/-/*` | 间隔运算 |

---

## 11.2 Rate（C++）

`Rate` 在循环末尾调用 `Sleep()`，**自动扣除本轮已耗时间**，使平均频率趋近目标值。适合 Binary 模式主循环稳频发布。

```cpp
#include "autolink/time/rate.hpp"

using autolink::Rate;

Rate rate(1.0);                  // 1 Hz
// Rate rate(100ms_as_nanoseconds);
// Rate rate(Duration(0.5));     // 2 Hz

while (autolink::OK()) {
    writer->Write(msg);
    rate.Sleep();                // 睡至下一周期边界
}
```

**talker.cpp 完整片段**：

```cpp
#include "autolink/time/rate.hpp"
#include "autolink/time/time.hpp"

Rate rate(1.0);
uint64_t seq = 0;
while (autolink::OK()) {
    auto msg = std::make_shared<Chatter>();
    msg->set_timestamp(Time::Now().ToNanosecond());
    msg->set_seq(seq++);
    talker->Write(msg);
    rate.Sleep();
}
```

| API | 说明 |
|-----|------|
| `Rate(double frequency)` | 目标频率（Hz） |
| `Rate(uint64_t nanoseconds)` | 目标周期（纳秒） |
| `Rate(const Duration&)` | 由间隔构造 |
| `Sleep()` | 等待至下一周期 |
| `Reset()` | 重置起始时刻 |
| `CycleTime()` | 上一轮实际周期 |
| `ExpectedCycleTime()` | 期望周期 |

若单轮 `Write` + 处理超过期望周期，`Sleep()` 立即返回，不会累积负睡眠。

---

## 11.3 Clock（C++）

`Clock` 单例统一 `Time::Now()` 的数据源，支持**系统时钟**与 **Mock 时钟**（仿真 / 单测）。

```cpp
#include "autolink/time/clock.hpp"

using autolink::Clock;
using autolink::proto::ClockMode;

// 默认 MODE_AUTOLINK：系统墙钟
Time t = Clock::Now();
double s = Clock::NowInSeconds();

// 仿真 / 测试：切换 Mock 并手动推进
Clock::SetMode(ClockMode::MODE_MOCK);
Clock::SetNow(Time(1000.0));
Clock::SetNowInSeconds(1001.5);
```

| `ClockMode` | 含义 |
|-------------|------|
| `MODE_AUTOLINK` | 系统时间（默认） |
| `MODE_MOCK` | 由 `SetNow` 设定，用于仿真回放 |

`run_mode_conf.proto` 中 `RunModeConf.clock_mode` 可在配置层指定；`RunMode::MODE_SIMULATION` 常与 Mock 时钟联用。

---

## 11.4 Timer（C++ API）

基于**时间轮**（`timing_wheel.hpp`），周期单位 **毫秒**，与 Time/Rate 独立。

```cpp
#include "autolink/timer/timer.hpp"

struct TimerOption {
    uint32_t period;              // ms，范围 1 ~ 512*64
    std::function<void()> callback;
    bool oneshot;                 // true=单次，false=周期
};

autolink::Timer timer(100, []() { AINFO << "tick"; }, false);
timer.Start();
// ...
timer.Stop();
```

亦可先默认构造再 `SetOption`：

```cpp
Timer t;
t.SetOption(TimerOption(50, callback, false));
t.Start();
```

回调在时间轮线程触发，不在 Reader 回调线程；仍不宜执行过重逻辑。

---

## 11.5 TimerComponent

继承 `TimerComponent`，实现无参 `Proc()`，由 DAG `interval`（ms）驱动：

**头文件**（`timer_component_example.hpp`）：

```cpp
class TimerComponentSample : public TimerComponent {
public:
    bool Init() override;
    bool Proc() override;
private:
    std::shared_ptr<Writer<Driver>> driver_writer_ = nullptr;
};
AUTOLINK_REGISTER_COMPONENT(TimerComponentSample)
```

**实现**（`timer_component_example.cpp`）：

```cpp
bool TimerComponentSample::Init() {
    driver_writer_ = node_->CreateWriter<Driver>("/carstatus/channel");
    return true;
}

bool TimerComponentSample::Proc() {
    static int i = 0;
    auto out_msg = std::make_shared<Driver>();
    out_msg->set_msg_id(i++);
    driver_writer_->Write(out_msg);
    return true;
}
```

**DAG**（`timer.dag`），`interval` 单位为 ms：

```text
timer_components {
    class_name : "TimerComponentSample"
    config {
        name : "timer"
        interval : 10
    }
}
```

```bash
mainboard -d autolink/examples/cpp/timer_component_example/timer.dag
```

---

## 11.6 机制对比

| 机制 | 线程模型 | 典型场景 | 示例 |
|------|----------|----------|------|
| `Time::Now()` | — | 消息时间戳、超时判断 | `talker.cpp` |
| `Duration::Sleep()` | 阻塞当前线程 | 简单延时 | `service.cpp` 中 `sleep(1)` |
| `Rate::Sleep()` | 阻塞当前线程，补偿耗时 | Binary 主循环稳频 | `talker.cpp` |
| `Timer` | 时间轮回调线程 | 独立周期任务 | 辅助逻辑 |
| `TimerComponent` | Scheduler 调度 | 无输入周期发布 | `timer_component_example` |

Component 体系内周期任务优先 **TimerComponent**；Binary 工具用 **Rate**；仅需时间戳用 **Time**，无需 Timer。

---

## 11.7 Python（Time / Rate）

`autolink_py3.autolink_time`（或 `from autolink_py3 import autolink_time`）提供与 C++ 对等的 Time / Duration / Rate。

**Time**（`py_time.py`）：

```python
from autolink_py3 import autolink_time

ct = autolink_time.Time(123)
print(ct.to_nsec())

# 墙钟
ftime = autolink_time.Time.now().to_sec()

# 单调时钟
mtime = autolink_time.Time.mono_time().to_sec()

# 运算
td1 = autolink_time.Duration(111)
tm1 = ct - td1
tm5 = autolink_time.Time(1.8)
```

**Duration**：

```python
td1 = autolink_time.Duration(111)              # 纳秒
td2 = autolink_time.Duration(601000000000)
td3 = td2 - td1
print(td2.to_sec(), td2.iszero())
td5 = autolink_time.Duration(1.8)            # 秒
```

**Rate**（`py_talker.py` 稳频发布）：

```python
rate = autolink_time.Rate(1.0)               # 1 Hz
# rate = autolink_time.Rate(0.2)             # 0.2 Hz
# rate = autolink_time.Rate(autolink_time.Duration(666))

while not autolink.is_shutdown():
    msg.timestamp = autolink_time.Time.now().to_nsec()
    writer.write(msg)
    rate.sleep()
```

运行示例：

```bash
python3 autolink/examples/python/py_time.py
```

---

## 11.8 Python（Timer）

`autolink_py3.timer` 对应 C++ `Timer`（`py_timer.py`）：

```python
from autolink_py3 import timer as autolink_timer

def fun():
    print("cb fun is called:", count)

autolink.init()
ct = autolink_timer.Timer(10, fun, 0)   # 10ms, oneshot=0 表周期
ct.start()
time.sleep(1)
ct.stop()

ct2 = autolink_timer.Timer()
ct2.set_option(10, fun, 0)
ct2.start()
```

---

## 11.9 选型

| 需求 | 推荐 |
|------|------|
| 消息打时间戳 | `Time::Now().ToNanosecond()` |
| 测执行耗时 | `Time::MonoTime()` 作起止 |
| Binary 稳频发布 | `Rate` |
| 仿真 / 单测时间 | `Clock::SetMode(MODE_MOCK)` |
| 周期发布、无输入 channel | TimerComponent + DAG |
| 独立周期回调 | `Timer` |
| 多路传感器对齐 | [Component](10_component.md) |

---

## 11.10 排错

| 现象 | 处理 |
|------|------|
| 时间戳跳变 | 墙钟受 NTP 影响；测间隔用 `MonoTime` |
| Rate 频率偏低 | 单轮耗时超过周期；优化回调或降频 |
| Timer 不触发 | 是否 `Start()`；period 是否在 1~32768 ms |
| TimerComponent 无输出 | `Init` 是否创建 Writer；DAG `interval` 是否过小 |
| Python 导入失败 | 设置 `PYTHONPATH`（见 `examples/python/README.md`） |
| Mock 时间不前进 | 仿真需手动 `Clock::SetNow` 或由仿真器驱动 |

---

**导航**：[← §10 Component](10_component.md) · [§0 指南](00_guide.md) · [§12 Scheduler →](12_scheduler.md)
