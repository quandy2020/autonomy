# Safety domain

L4 只闩锁：`safety_guard` 置位后保持到人工 `Clear()`。不改写 `/cmd_vel`，采集进程不插拦截器。闸门 `AUTOLOOP_LATCH_ONLY`。

安全事件追加 SHA-256 链（`hash_util`）；云端校验失败 → `safety_event` 告警且该 robot 制品不得晋升。MRM / 制动仍由 `autonomy.monitor` 执行。
