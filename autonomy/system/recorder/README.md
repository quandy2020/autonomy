# autonomy.recorder — 数据闭环证据链

旁路进程：订阅 autolink，不修改 planning / control / perception。MCAP 容器 + Protobuf 载荷；缺版本四元组进诊断队列，不进训练。

长文：[`../docs/recorder/`](../docs/recorder/)（需求 / 方案 / 架构 / 数据流 / 使用）。

开关：`-DAUTONOMY_BUILD_RECORDER=ON`（默认 ON）。

```bash
autonomy.recorder --conf=recorder.pb.txt
autonomy.recorder --conf=recorder.pb.txt --full-dump
autonomy.recorder safety --conf=recorder.pb.txt
autonomy.recorder replay -i episode.mcap -r 1 --profile open_loop
autonomy.recorder replay -i episode.mcap --profile mid_stack
autonomy.recorder replay -i episode.mcap --hil
autonomy.recorder gate -d ~/.autonomy/records
autonomy.recorder convert -i episode.mcap -o /tmp/lerobot
autonomy.recorder pipeline -i episode.sidecar.pb
autonomy.recorder verdict -a rec.mcap -b replay.mcap -o overlay.json
autonomy.recorder observe -o ep.observations.jsonl --id o1 -d "desc" -n node_id
autonomy.recorder upload -s ~/.autonomy/records/episodes -d /data/inbox
autonomy.recorder verify-chain -i ~/.autonomy/records/safety/chain.bin
```

四元组：`policy_version` / `env_version` / `git_sha` / `seed`。双时钟：monotonic + UTC。时间源默认 software。

## 模块

| 路径 | 层 | 作用 |
|------|----|------|
| `subscriber` | L1 | RawMessage 旁路，BEST_EFFORT，写预序列化字节 |
| `rolling_buffer` + `trigger` + `episode` | L1 | 滚动缓冲；急停 / BT recover / 任务 feedback / 定位质量 / 路径偏离 / 丢包 / 接管；`--full-dump` 会话落盘 |
| `signals` | L1 | 触发解析 odom/plan/twist/cost/task feedback（不改写控制） |
| `mcap_writer` | L1 | append-only MCAP，崩溃可 `RecoverMcap` |
| `value_tier` + `upload` | 边 | P0 立即上传；P1 上传非专家；P2/P3 分层抽样；`upload` 按 manifest 拷贝 |
| `event_graph` + `root_cause` | L2 | 事件图、根因卡片、re-roll |
| `bt_attribution` | L3 | 失败定位到 BT 节点 |
| `hash_chain` + `safety_monitor` | L4 | 哈希链；独立进程只闩 `SafetyLatch`，不发 `/cmd_vel` |
| `metrics` | L5 | 机队 jsonl 上 MPS/MPD、尾部、`gate/latest.json` |
| `replay` + `player` | L6 | 动作匹配；`open_loop` / `mid_stack` / `sensor_in`；`--hil` 1x |
| `convert` | 边 | MCAP → LeRobot v2.1 JSONL |
| `sidecar` / `observation` | autoviz | JSON sidecar（含 graph_edges）；无 evidence_node_id 不得进 L2 |
| `pipeline` | L1→L6 | 单 episode 走完整条链 |

落盘：`$output_dir/episodes/<id>.mcap` + `.sidecar.pb/.json` + `.manifest.json` + `.overlay.json`；四元组不齐 → `diagnostics/`。观察写回 `.observations.jsonl`，必须绑定 `evidence_node_id`。

独立安全进程：`autolink_launch safety.launch`（`autonomy.recorder safety`）。超速只置位 `SafetyLatch`，由既有 monitor MRM 停机。

autoviz：Add Panel → **Evidence**，打开 `*.sidecar.json`；时间轴 / 因果图 / BT / 观察写回 / overlay 对比共用 cursor，可 seek Record 回放。不链 `autonomy_system`。

参数回写仍走 gRPC Bridge，且仅在 episode 之间应用。
