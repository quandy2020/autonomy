# 8. Log

autolink 日志基于 **glog**，宏定义于 `autolink/common/log.hpp`。

| 本文 §8 | 相关文档 |
|---------|----------|
| 日志 | [§0 指南](00_guide.md) · [§12 Scheduler](12_scheduler.md) |

每条日志附带 **模块名** `MODULE_NAME`（默认二进制名）。`Init()` 时初始化 glog。

---

## 8.1 基本用法

```cpp
#include "autolink/autolink.hpp"   // 间接包含 log.hpp

AINFO << "talker sent a message! No. " << seq;
AWARN << "service may not ready.";
AERROR << "failed to create service/client.";
ADEBUG << "verbose debug";   // VLOG(4)
```

输出格式：`[module_name][LEVEL] file:line message`。

---

## 8.2 宏一览

| 宏 | glog 级别 | 说明 |
|----|-----------|------|
| `ADEBUG` | VLOG(4) | 需提高 verbose |
| `AINFO` | INFO | 常规信息 |
| `AWARN` | WARNING | 可恢复异常 |
| `AERROR` | ERROR | 错误 |
| `AFATAL` | FATAL | 记录后终止进程 |
| `AINFO_IF(cond)` | 条件 INFO | |
| `AINFO_EVERY(n)` | 每 n 次打印 | 热路径采样 |
| `ACHECK(cond)` | CHECK | 失败则 abort |

定义片段（`log.hpp`）：

```cpp
#define AINFO ALOG_MODULE(MODULE_NAME, INFO)
#define AWARN ALOG_MODULE(MODULE_NAME, WARN)
#define AERROR ALOG_MODULE(MODULE_NAME, ERROR)
```

自定义模块名：编译前 `#define MODULE_NAME "my_module"`。

---

## 8.3 环境变量（glog）

| 变量 | 作用 |
|------|------|
| `GLOG_log_dir` | 日志文件目录 |
| `GLOG_v` | 全局 VLOG 级别（`ADEBUG` 可见性） |
| `GLOG_minloglevel` | 最低级别：0=INFO, 1=WARNING, 2=ERROR |
| `GLOG_stderrthreshold` | 复制到 stderr 的最低级别 |

示例：

```bash
export GLOG_log_dir=/tmp/autolink_logs
export GLOG_v=4
./autolink_example_talker
```

---

## 8.4 与 Scheduler 的关系

异步日志使用独立线程 `async_log`。在 `example_sched_classic.conf` 中可为其绑核，避免磁盘 I/O 抢占实时回调：

```text
threads: [{
    name: "async_log"
    cpuset: "1"
    policy: "SCHED_OTHER"
    prio: 0
}]
```

见 [§12 Scheduler](12_scheduler.md)。

---

## 8.5 Autonomy 应用层

`autonomy/common/logging.hpp` 提供与 autolink 风格一致的封装，Server 模块（`planner_server.cpp` 等）中 `AWARN` / `AERROR` 与 autolink 宏混用。保持**热路径少打日志**：Reader 回调、`Component::Proc` 内优先 `AINFO_EVERY` 或采样。

---

## 8.6 实践建议

```cpp
// 不推荐：高频回调内无条件 INFO
void OnImu(const std::shared_ptr<Imu>& msg) {
    AINFO << "imu " << msg->seq();
}

// 推荐：采样
AINFO_EVERY(100) << "imu still alive";
```

- 错误路径：`AERROR` + 明确返回，便于与 `autolink_recorder` 联调。  
- 录包场景：日志时间戳与消息 `timestamp` 对照定位延迟。

---

## 8.7 排错

| 现象 | 处理 |
|------|------|
| 无日志输出 | 检查 `GLOG_minloglevel`；是否未 `Init` |
| 日志拖慢实时线程 | 降低频率；配置 `async_log` 绑核 |
| ADEBUG 不可见 | `export GLOG_v=4` |

---

**导航**：[← §7 Plugin](07_plugin.md) · [§0 指南](00_guide.md) · [§9 Launch →](09_launch.md)
