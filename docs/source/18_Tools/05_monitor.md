# 5. 系统监控

路径：`autonomy/system/monitor/`。配置：`config/system/monitor.lua` 或对应 pb。

**什么时候用：** 看 CPU/内存/磁盘/通道延迟、Prometheus 抓取，或排查「机载很卡」。

### 开关（概念）

| 字段 | 含义 |
|------|------|
| `enable_cpu/mem/hdd/net/ntp/process_monitor` | 主机指标 |
| `enable_gpu_monitor` | DRM GPU |
| `enable_channel/latency/hazard_monitor` | Autolink 通道 |
| `enable_prometheus` | 暴露 `/metrics` |
| `collect_interval_sec` | 采集周期 |

### 怎么跑

```bash
autonomy.monitor --conf=monitor.pb.txt
# 或 autolink_launch monitor.launch / autonomy.launch 一并起
```

Prometheus：`BUILD_PROMETHEUS=ON`。细节见源码 `monitor/README.md`。

Bridge `GetHealth` 字段与 monitor 概念对齐（见 `system.proto` 注释）。

### 常见坑

| 现象 | 说明 |
|------|------|
| metrics 空 | prometheus 开关未开或端口未映射 |
| hazard 误报 | channel 名与 launch 不一致 |
