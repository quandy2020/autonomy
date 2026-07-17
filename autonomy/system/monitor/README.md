# System Monitor

基于 **gperftools (gperf)** 与 **prometheus-cpp** 的系统监控模块，支持**可视化**与**参数选定**。CPU/内存等核心项通过 Linux `/proc` 与 `sysfs` 采集，**同一套实现**适用于 x86_64 与 aarch64（不再按平台拆分空壳 monitor 类）。

## 功能概览

- **参数选定**：通过 `MonitorOptions` 选择启用的监控项（CPU / GPU / 内存 / 磁盘 / 网络 / NTP / 进程 / 电压）以及是否开启 Prometheus 暴露、gperf CPU/堆分析。
- **Prometheus 可视化**：启用后在本机暴露 HTTP metrics（默认 `0.0.0.0:9090`），可用 Prometheus 抓取、Grafana 绘图。
- **gperf 分析**：可选开启 CPU 分析（`.prof`）与堆分析，便于性能调优。

## 依赖

- **BUILD_PROMETHEUS=ON** 时需安装 **prometheus-cpp**（core + pull）。
- **gperftools** 为可选：未安装时 CPU/堆分析接口为空实现，其余监控不受影响。

## 使用方式

### 1. 配置选项（参数选定）

**Lua**（推荐）：编辑 `config/system/monitor.lua`，由 `LoadMonitorOptions(dir, basename)` 加载。

**独立进程**：

```bash
monitor --configuration_directory=config --configuration_basename=system/monitor.lua
```

**代码**：

```cpp
#include "autonomy/system/monitor/monitor_options.hpp"
#include "autonomy/system/monitor/monitor_registry.hpp"

using namespace autonomy::system::monitor;

MonitorOptions opts = LoadMonitorOptions("config", "system/monitor.lua");
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
| `autonomy_system_cpu_temperature_celsius{sensor="..."}` | CPU 温度（coretemp 或 thermal_zone，有则暴露） |
| `autonomy_system_load_average{period="1m\|5m\|15m"}` | 系统负载 |
| `autonomy_system_mem_usage_percent` | 内存使用率 |
| `autonomy_system_mem_total_kb` | 总内存 (KB) |
| `autonomy_system_mem_available_kb` | 可用内存 (KB) |
| `autonomy_system_mem_swap_used_kb` | 已用 Swap (KB) |
| `autonomy_system_disk_usage_percent{mount="..."}` | 各挂载点磁盘使用率 |
| `autonomy_system_net_rx_bytes_per_second{device="..."}` | 网卡接收速率 |
| `autonomy_system_net_tx_bytes_per_second{device="..."}` | 网卡发送速率 |
| `autonomy_system_process_count` | 进程数 |
| `autonomy_system_process_rss_total_kb` | 进程 VmRSS 合计 (KB) |
| `autonomy_system_ntp_offset_seconds` | chronyc 时钟偏移 (秒) |
| `autonomy_system_hazard_level` | 0/1/2（OK/WARN/ERROR） |
| `autonomy_system_mrm_active` | MRM 急停是否在发零 `cmd_vel` |
| `autonomy_system_channel_ok{channel}` | 通道是否在超时内收到数据 |
| `autonomy_system_pipeline_latency_seconds{channel}` | 消息 header 年龄 |

其他 monitor（voltage）仍预留接口；无 NVML / SMART 专用守护进程时不重复实现 Autoware 侧车方案。

## 构建

- 开启 Prometheus：  
  `cmake -DBUILD_PROMETHEUS=ON ...`  
  并确保已安装 prometheus-cpp（如 vcpkg 或系统包）。
- gperftools：若需 CPU/堆分析，安装 gperftools 即可；不安装也可正常编译运行，分析接口为空。
