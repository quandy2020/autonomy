# 5. 系统监控

`autonomy/system/monitor/` 通过 `/proc`、`sysfs` 采集 CPU、内存、磁盘、网络、进程、NTP 等指标；可选 Prometheus 暴露与 gperftools 分析。**x86_64 / aarch64 同一套实现**。

### 5.1 配置（Lua）

默认文件：`config/system/monitor.lua`。

| 监控项 | 选项字段 |
|--------|----------|
| CPU | `enable_cpu_monitor` |
| GPU（DRM `gpu_busy_percent`） | `enable_gpu_monitor` |
| 内存 | `enable_mem_monitor` |
| 磁盘 | `enable_hdd_monitor` |
| 网络 | `enable_net_monitor` |
| NTP（`chronyc`） | `enable_ntp_monitor` |
| 进程 | `enable_process_monitor` |
| Channel 健康 | `enable_channel_monitor`、`channel_watches` |
| 管线延迟 | `enable_latency_monitor`、`latency_watches`（TwistStamped header 年龄） |
| Hazard | `enable_hazard_monitor` |
| MRM 急停 | `enable_mrm_handler`（默认关，ERROR 时发零 `cmd_vel`） |
| 电压 | `enable_voltage_monitor`（预留） |
| Prometheus | `enable_prometheus`、`prometheus_bind_address` |
| 采集周期 | `collect_interval_sec` |
| gperf | `enable_cpu_profile` / `enable_heap_profile` |

### 5.2 独立进程

```bash
monitor --configuration_directory=config --configuration_basename=system/monitor.lua
```

或通过 launch：

```bash
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/system/launch
autolink_launch monitor.launch
```

与导航栈一并启动：`autonomy/system/launch/autonomy.launch`。

### 5.3 嵌入代码

```cpp
#include "autonomy/system/monitor/monitor_options.hpp"
#include "autonomy/system/monitor/monitor_registry.hpp"

using namespace autonomy::system::monitor;

auto opts = LoadMonitorOptions("config", "system/monitor.lua");
MonitorRegistry registry(opts);
registry.Start();
registry.CollectAll();  // 或由定时线程周期性调用
```

### 5.4 构建选项

| CMake 选项 | 说明 |
|------------|------|
| `BUILD_PROMETHEUS=ON` | 启用 prometheus-cpp，暴露 `/metrics` |

gperftools 为可选；未安装时 CPU/堆分析为空实现。

### 5.5 相关文档

- 源码 README：`autonomy/system/monitor/README.md`
