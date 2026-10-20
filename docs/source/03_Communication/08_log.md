# 8. 日志 Log

两层：应用宏（`autolink/common/log.hpp`）+ 异步落盘（`AsyncLogger`）。`Init()` 初始化 glog 并注册 `async_log` 线程。

## 8.1 宏

```cpp
#include "autolink/autolink.hpp"
AINFO << "PlannerServer started.";
AWARN << "service may not ready.";
AERROR << "CreateService failed.";
ADEBUG << "debug detail"; // 需 GLOG_v≥4
```

| 宏 | 用途 |
|----|------|
| `ADEBUG` / `AINFO` / `AWARN` / `AERROR` / `AFATAL` | 级别 |
| `AINFO_IF` / `AINFO_EVERY(n)` | 条件 / 采样 |
| `ACHECK` | 断言 |
| `RETURN_IF_NULL` / `RETURN_IF` | 早返回 + AWARN |

自定义模块名：`#define MODULE_NAME "planning"` 后再 `#include` log.hpp。Autonomy 层见 `autonomy/common/logging.hpp`。

## 8.2 GLOG

| 变量 | 含义 |
|------|------|
| `GLOG_logtostderr` | `1` → 打 stderr |
| `GLOG_minloglevel` | 0=INFO … |
| `GLOG_log_dir` | 落盘目录 |
| `GLOG_v` | VERBOSE（`ADEBUG`） |

落盘路径形如：`$GLOG_log_dir/{module}.log.INFO.{timestamp}.{pid}`。

## 8.3 回调内日志

热路径勿每帧 `AINFO`；用 `AINFO_EVERY(n)` 或降到 `ADEBUG`。
