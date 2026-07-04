# 8. 日志 Log

autolink 日志分两层：**应用宏**（`autolink/common/log.hpp`）与 **异步落盘**（`autolink/logger/`）。`Init()` 在 `init.cpp` 中初始化 glog、安装 `AsyncLogger`，并将写盘线程注册为 Scheduler 内部线程 `async_log`。对标 ROS 2 `RCLCPP_*`。

| 本文 §8 | 相关文档 |
|---------|----------|
| 宏、落盘、绑核 | [§0 指南](00_guide.md) · [§12 调度 Scheduler](12_scheduler.md) · [§3 通道 Channel](03_channel.md) |

| 源码 | 职责 |
|------|------|
| `common/log.hpp` | `AINFO` / `AWARN` / `AERROR` 等宏；`MODULE_NAME` |
| `logger/async_logger.hpp` | 双缓冲异步写盘（生产路径） |
| `logger/logger.hpp` | 按模块名分文件的同步 Logger（测试 / 定制） |
| `logger/log_file_object.hpp` | 单模块日志文件、滚动 |
| `logger/logger_util.hpp` | 从日志行解析 `[module]` |
| `init.cpp` | `InitLogger()` · `async_log` 线程注册 |

---

## 8.1 日志架构

<div class="comm-flow-diagram">
<div class="comm-flow-header">
  <span class="comm-flow-badge">落盘路径</span>
  <span class="comm-flow-title">AINFO → glog → AsyncLogger → async_log → LogFileObject</span>
  <span class="comm-flow-sub">业务线程在 <code>AsyncLogger::Write</code> 入队即返回；写盘由 Scheduler 线程 <code>async_log</code> 批量完成</span>
</div>
<div class="comm-flow-pipeline comm-flow-pipeline--chain">
  <div class="comm-flow-step comm-flow-step-client">
    <span class="comm-flow-step-label">①</span>
    <span class="comm-flow-step-title">AINFO</span>
    <span class="comm-flow-step-sub">log.hpp · [MODULE_NAME]</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-label">②</span>
    <span class="comm-flow-step-title">LogMessage</span>
    <span class="comm-flow-step-sub">glog</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-label">③</span>
    <span class="comm-flow-step-title">AsyncLogger::Write</span>
    <span class="comm-flow-step-sub">双缓冲入队</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-label">④</span>
    <span class="comm-flow-step-title">async_log</span>
    <span class="comm-flow-step-sub">RunThread · FlushBuffer</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-server">
    <span class="comm-flow-step-label">⑤</span>
    <span class="comm-flow-step-title">LogFileObject</span>
    <span class="comm-flow-step-sub">{module}.log.INFO.*</span>
  </div>
</div>
<div class="comm-flow-foot">
  <div class="comm-flow-chips">
    <span class="nav-chip">FindModuleName 路由</span>
    <span class="nav-chip">InitLogger · init.cpp</span>
    <span class="nav-chip">见 §8.5 async_log 绑核</span>
  </div>
</div>
</div>

**模块名解析**：宏在消息头写入 `[module]`；`FindModuleName()`（`logger_util.hpp`）解析后路由到对应 `LogFileObject`。默认 `MODULE_NAME` 为 `autolink::binary::GetName()`（`Init(argv[0])` 时设为二进制名）。

**落盘路径**（`AsyncLogger::FlushBuffer`）：

```
$GLOG_log_dir/{module}.log.INFO.{timestamp}.{pid}
```

未设置 `GLOG_log_dir` 时写入当前工作目录；同时可通过 `GLOG_stderrthreshold` 复制到终端。

**与原生 glog 差异**：WARNING 及以上不再同步 flush 到磁盘，而由 `async_log` 线程批量写入；`FATAL` 仍会刷缓冲后退出。缓冲区满时生产者可能阻塞，防止内存失控（见 `async_logger.hpp` 注释）。

---

## 8.2 宏

`autolink/common/log.hpp`：`AINFO` 等级别宏、`[MODULE_NAME]` 前缀、`RETURN_IF_*` 早返回守卫。

```cpp
#include "autolink/autolink.hpp"   // 间接包含 log.hpp

AINFO  << "PlannerServer started.";
AWARN  << "service may not ready.";
AERROR << "CreateService failed.";
ADEBUG << "debug detail";          // VLOG(4)，需 GLOG_v≥4
```

终端一行格式：`I{date} {time} {pid} {file}:{line}] [module] message`。

| 宏 | glog | 用途 |
|----|------|------|
| `ADEBUG` | VLOG(4) | 调试；默认不可见 |
| `AINFO` | INFO | 状态变更、低频事件 |
| `AWARN` | WARNING | 可恢复异常 |
| `AERROR` | ERROR | 错误；须伴随 `return` / 错误码 |
| `AFATAL` | FATAL | 不可恢复；终止进程 |
| `AINFO_IF` / `AWARN_IF` / … | — | 条件打印 |
| `AINFO_EVERY(n)` | LOG_EVERY_N | 每 n 次打印（回调采样） |
| `ACHECK(cond)` | CHECK | 断言失败 abort |

**守卫宏**（`log.hpp`，早返回并打 `AWARN`）：

| 宏 | 行为 |
|----|------|
| `RETURN_IF_NULL(ptr)` | `ptr==nullptr` → AWARN + return |
| `RETURN_VAL_IF_NULL(ptr, val)` | 同上，返回 `val` |
| `RETURN_IF(cond)` | 条件成立 → AWARN + return |
| `RETURN_VAL_IF(cond, val)` | 同上，返回 `val` |

**自定义模块名**：在 `#include "autolink/common/log.hpp"` 之前定义：

```cpp
#define MODULE_NAME "planning"
#include "autolink/common/log.hpp"
```

Autonomy 应用层等价封装见 `autonomy/common/logging.hpp`（默认 `MODULE_NAME "autonomy"`），Server 代码可与 autolink 宏混用。

---

## 8.3 使用案例

### Binary：Talker 稳频发布

`autolink/examples/cpp/talker.cpp` — 循环内记录发送序号：

```cpp
autolink::Init(argv[0]);
// ...
while (autolink::OK()) {
    talker->Write(msg);
    AINFO << "talker sent a message! No. " << seq;
    ++seq;
    rate.Sleep();
}
```

### Channel：Listener 回调

`listener.cpp` — 每条消息 INFO（**仅示例**；生产环境应采样，见 [§8.6](#86-回调内日志)）：

```cpp
void MessageCallback(const std::shared_ptr<Chatter>& msg) {
    AINFO << "Received message seq-> " << msg->seq();
}
```

### Service：创建失败

`service.cpp`：

```cpp
if (server == nullptr || client == nullptr) {
    AERROR << "failed to create service/client.";
    return 1;
}
```

### Action：超时与拒绝

`action_talker.cpp`：

```cpp
opts.goal_response_callback = [](std::shared_ptr<GoalHandle> gh) {
    if (!gh) AWARN << "Goal rejected by server";
};
// ...
if (accepted_future.wait_for(kAcceptTimeout) == std::future_status::timeout) {
    AERROR << "Timeout waiting for goal acceptance";
    return 1;
}
```

### Record：读包失败

`record.cpp`：

```cpp
if (!reader.ReadMessage(&msg)) {
    AERROR << "read msg[" << i << "] failed";
}
```

### 查看落盘文件

```bash
export GLOG_log_dir=/tmp/autolink_logs
./autolink_example_talker &
./autolink_example_listener
ls /tmp/autolink_logs/    # autolink_example_talker.log.INFO.* 等
```

---

## 8.4 GLOG 配置

| 变量 / Flag | 作用 |
|-------------|------|
| `GLOG_log_dir` | 日志文件目录（`AsyncLogger` 落盘根路径） |
| `GLOG_v` | VLOG 级别；`≥4` 时 `ADEBUG` 可见 |
| `GLOG_minloglevel` | 最低级别：`0` INFO · `1` WARNING · `2` ERROR |
| `GLOG_stderrthreshold` | 复制到 stderr 的最低级别 |
| `FLAGS_max_log_size` | 单文件上限（MB），默认见 glog |

```bash
export GLOG_log_dir=/tmp/autolink_logs
export GLOG_v=4
export GLOG_stderrthreshold=2    # ERROR 及以上同时打终端
./autolink_example_talker
```

---

## 8.5 async_log 线程

`Init()` 内（`init.cpp`）：

```cpp
async_logger = new logger::AsyncLogger(google::base::GetLogger(FLAGS_minloglevel));
google::base::SetLogger(FLAGS_minloglevel, async_logger);
async_logger->Start();
scheduler::Instance()->SetInnerThreadAttr("async_log", async_logger->LogThread());
```

Scheduler 配置中为 `async_log` 单独绑核，避免磁盘 I/O 与 Reader / `Proc` 抢 CPU（[§12 classic](12_scheduler.md#123-classic)）：

```text
threads: [{
    name: "async_log"
    cpuset: "1"
    policy: "SCHED_OTHER"
    prio: 0
}]
```

---

## 8.6 回调内日志

Reader 回调与 `Component::Proc` 运行在 Scheduler 线程；无条件高频 `AINFO` 会拉长回调、引发队列堆积（[§3](03_channel.md) · [§12](12_scheduler.md#125-联调要点)）。

| 场景 | 建议 |
|------|------|
| 传感器 / 控制环 | `AINFO_EVERY(100)` 或仅状态变更时打印 |
| 可恢复错误 | `AWARN` + 继续 / 降级 |
| 不可恢复 | `AERROR` + 明确 `return false` |
| 录包联调 | 日志时间与消息 `timestamp` 对照 |

```cpp
// 不推荐：每帧 INFO（listener 示例仅用于学习）
void OnChatter(const std::shared_ptr<Chatter>& msg) {
    AINFO << "seq=" << msg->seq();
}

// 推荐
AINFO_EVERY(100) << "chatter stream alive";
if (msg->seq() == 0) AWARN << "unexpected seq=0";
```

---

**导航**：[← §7 插件 Plugin](07_plugin.md) · [§0 指南](00_guide.md) · [§9 启动 Launch →](09_launch.md)
