# 5. 系统监控

`autonomy/system/monitor/` 提供 CPU、GPU、内存、磁盘、网络、NTP、进程、电压等监控，可选 Prometheus 暴露与 gperftools 性能分析。

### 5.1 功能概览

| 监控项 | 选项字段 |
|--------|----------|
| CPU | `enable_cpu_monitor` |
| GPU | `enable_gpu_monitor` |
| 内存 | `enable_mem_monitor` |
| 磁盘 | `enable_disk_monitor` |
| 网络 | `enable_network_monitor` |
| Prometheus | `enable_prometheus`（默认 `0.0.0.0:9090`） |
| gperf CPU/堆分析 | `enable_cpu_profile` / `enable_heap_profile` |

### 5.2 基本用法

```cpp
#include "autonomy/system/monitor/monitor_options.hpp"
#include "autonomy/system/monitor/monitor_registry.hpp"

using namespace autonomy::system::monitor;

MonitorOptions opts = MonitorOptions::Default();
opts.enable_cpu_monitor = true;
opts.enable_mem_monitor = true;
opts.enable_prometheus = true;
opts.prometheus_bind_address = "0.0.0.0:9090";

MonitorRegistry registry(opts);
registry.Start();
```

### 5.3 构建选项

| CMake 选项 | 说明 |
|------------|------|
| `BUILD_PROMETHEUS=ON` | 启用 prometheus-cpp 依赖 |

gperftools 为可选；未安装时 CPU/堆分析为空实现。

### 5.4 相关文档

- 源码 README：`autonomy/system/monitor/README.md`
