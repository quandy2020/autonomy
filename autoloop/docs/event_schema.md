# Event schema

JSONL / proto 一行一事，禁止二进制 payload（通道字节在 MCAP）。

| 字段 | 用途 |
|------|------|
| `ts_mono_ns` / `ts_utc_ns` | 双时钟；回放只用 mono |
| `trace_id` / `span_id` | 回溯到 episode |
| `event_code` | heartbeat / snapshot / upload / latch / drop / crash / model_rollback |
| `severity` | 仪表密度 |
| `robot_id` / `episode_id` | 主键 |
| `payload` | JSON object |
