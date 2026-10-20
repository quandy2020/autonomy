# 8. 日志 Log

`AINFO` / `AWARN` 等宏怎么用、`GLOG_*` 怎么设。`Init()` 会初始化 glog 并挂异步落盘线程。

日志依赖 `Init()` 完成的 glog 与异步落盘线程；控制环排障时用 `GLOG_v` 压低框架噪声、抬高业务模块。关键状态不要只依赖日志尾行——高频率 channel 或 metrics 更可靠。

## 宏

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

## GLOG

| 变量 | 含义 |
|------|------|
| `GLOG_logtostderr` | `1` → 打 stderr |
| `GLOG_minloglevel` | 0=INFO … |
| `GLOG_log_dir` | 落盘目录 |
| `GLOG_v` | VERBOSE（`ADEBUG`） |

落盘路径形如：`$GLOG_log_dir/{module}.log.INFO.{timestamp}.{pid}`。

## 回调内日志

热路径勿每帧 `AINFO`；用 `AINFO_EVERY(n)` 或降到 `ADEBUG`。
