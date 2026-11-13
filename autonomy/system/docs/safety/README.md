# L4 安全证据

独立于 planning / control / perception。策略输出仍走 `/cmd_vel`；本进程只订阅、判定、闩锁。

- **通过** `EvaluateCommand` → Pass，不写链。
- **回退** `|vx|` / `|wz|` 超限 → `SafetyLatch`（`safety_monitor_cmd_limit`），monitor MRM 停机；事件进 SHA-256 哈希链。
- **移交** teleop goal → 记 `handover`，不闩锁。
- **不发布** `/cmd_vel`。已闩则 Pass，避免重复闩。

```bash
autolink_launch safety.launch
autonomy.recorder safety --conf=recorder.pb.txt
autonomy.recorder verify-chain -i ~/.autonomy/records/safety/chain.bin
```

链文件：`safety_chain_path` 或 `$output_dir/safety/chain.bin`。episode sidecar 的 `safety_events` 与此同源。急停文件闩仍是 `AUTONOMY_SAFETY_LATCH_PATH`。
