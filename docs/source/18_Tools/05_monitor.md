# 5. 系统监控

路径：`autonomy/system/monitor/`。配置：`config/system/monitor.lua`。

| 开关字段 | 含义 |
|----------|------|
| `enable_cpu/mem/hdd/net/ntp/process_monitor` | 基础指标 |
| `enable_gpu_monitor` | DRM GPU |
| `enable_channel/latency/hazard_monitor` | 通道 / 延迟 / 危险 |
| `enable_prometheus` | `/metrics` |
| `collect_interval_sec` | 周期 |

```bash
autonomy.monitor --conf=monitor.pb.txt
# 或 autolink_launch monitor.launch / autonomy.launch
```

CMake：`BUILD_PROMETHEUS=ON`。详见源码 `monitor/README.md`。
