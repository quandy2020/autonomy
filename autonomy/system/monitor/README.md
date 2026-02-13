# System Monitor

基于 **gperftools (gperf)** 与 **prometheus-cpp** 的系统监控模块，支持**可视化**与**参数选定**。

## 功能概览

- **参数选定**：通过 `MonitorOptions` 选择启用的监控项（CPU / GPU / 内存 / 磁盘 / 网络 / NTP / 进程 / 电压）以及是否开启 Prometheus 暴露、gperf CPU/堆分析。
- **Prometheus 可视化**：启用后在本机暴露 HTTP metrics（默认 `0.0.0.0:9090`），可用 Prometheus 抓取、Grafana 绘图。
- **gperf 分析**：可选开启 CPU 分析（`.prof`）与堆分析，便于性能调优。

## 依赖

- **BUILD_PROMETHEUS=ON** 时需安装 **prometheus-cpp**（core + pull）。
- **gperftools** 为可选：未安装时 CPU/堆分析接口为空实现，其余监控不受影响。

## 使用方式

### 1. 配置选项（参数选定）

```cpp
#include "autonomy/system/monitor/monitor_options.hpp"
#include "autonomy/system/monitor/monitor_registry.hpp"

using namespace autonomy::system::monitor;

MonitorOptions opts = MonitorOptions::Default();
opts.enable_cpu_monitor = true;
opts.enable_mem_monitor = true;
opts.enable_prometheus = true;
opts.prometheus_bind_address = "0.0.0.0:9090";
opts.enable_cpu_profile = false;
opts.enable_heap_profile = false;
opts.collect_interval_sec = 1.0;

MonitorRegistry registry(opts);
registry.Start();
```

### 2. 周期性采集与可视化

- 定时调用 `registry.CollectAll()`（例如按 `collect_interval_sec`）更新指标。
- 当 `enable_prometheus == true` 时，指标在 `prometheus_bind_address` 暴露，例如：
  - 打开 `http://<host>:9090/metrics` 查看 Prometheus 文本格式。
  - 在 Prometheus 中配置 scrape 该地址，再在 Grafana 中选 Prometheus 数据源做可视化。

### 3. gperf 分析（可选）

```cpp
opts.enable_cpu_profile = true;
opts.cpu_profile_filename = "/tmp/my_cpu.prof";
opts.enable_heap_profile = true;
opts.heap_profile_filename = "/tmp/my_heap.prof";
// 然后 Start() 会按选项自动启停分析；Stop() 时停止并写文件。
```

## 指标说明（Prometheus）

| 指标名 | 含义 |
|--------|------|
| `autonomy_system_cpu_usage_percent{cpu="total\|cpu0\|cpu1\|..."}` | CPU 使用率（整体或每核） |
| `autonomy_system_mem_usage_percent` | 内存使用率 |
| `autonomy_system_mem_total_kb` | 总内存 (KB) |
| `autonomy_system_mem_available_kb` | 可用内存 (KB) |
| `autonomy_system_gpu_usage_percent` | GPU 使用率（当前为 stub） |

其他 monitor（hdd/net/ntp/process/voltage）已预留接口，可按需扩展实现与指标。

## 构建

- 开启 Prometheus：  
  `cmake -DBUILD_PROMETHEUS=ON ...`  
  并确保已安装 prometheus-cpp（如 vcpkg 或系统包）。
- gperftools：若需 CPU/堆分析，安装 gperftools 即可；不安装也可正常编译运行，分析接口为空。
