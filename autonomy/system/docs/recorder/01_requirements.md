# 01 需求

将机上运行从「可回放的日志」提升为「可裁决的因果证据」：每个 episode 能回答失败是否可复现、根因落在哪一棵 BT 节点、安全事件是否可审计、候选策略是否允许发布。

## 目标

| 编号 | 陈述 |
|------|------|
| G1 | 在不改写规划 / 控制 / 感知的前提下，旁路捕获通道字节与双时钟戳。 |
| G2 | episode 绑定确定性版本四元组；缺失字段进入诊断队列，禁止进入训练与正式回放。 |
| G3 | 默认事件触发落盘（急停、BT recover、任务 feedback 失败、定位质量、路径偏离、超时、接管）。测试全量用 `--full-dump` 会话落盘。 |
| G4 | 离线产出事件图、BT 归因、根因卡片、安全哈希链、回放裁决与发布门禁。 |
| G5 | 边侧按 P0–P3 决定上传；MCAP 可转为 LeRobot v2.1 JSONL（当前帧为 odom+cmd_vel；图像/BEV 留在 MCAP）。 |
| G6 | autoviz 以 sidecar JSON 做分诊（时间轴 / 因果图 / BT / 观察 / overlay），观察必须绑定 `evidence_node_id`。 |

## 非目标

| 项 | 说明 |
|----|------|
| 修改控制图 | 不在 policy 与执行器之间插入 `/cmd_vel` 转发节点。 |
| PTP / gPTP 硬件时钟 | proto 预留 `TIME_SOURCE_PTP` / `GPTP`；实现默认 `TIME_SOURCE_SOFTWARE`。 |
| 云端训练流水 | `upload` 只按 manifest 拷贝文件，不调度训练作业。 |
| 在线改参 | 参数回写走既有 gRPC Bridge，且仅在 episode 之间应用。 |
| 依赖 `autonomy_system` 的 autoviz | Evidence 面板只读 JSON。 |

## 约束

1. **编译隔离**：`-DAUTONOMY_BUILD_RECORDER`（默认 ON）；关闭后 `autonomy_system` 不含 `recorder/`。
2. **进程隔离**：独立 binary `autonomy.recorder`；默认 `autonomy.launch` 以旁路模块拉起。
3. **不背压**：订阅 QoS 为 BEST_EFFORT + KEEP_LAST；`depth`/`queue` 为 0 时启发式（高频 32/64，大载荷 2/8）。
4. **容器与载荷**：MCAP 为容器；Protobuf 为载荷；写入已序列化字节，采集路径不反序列化业务消息（触发解析 odom / plan / twist / cost 除外）。
5. **安全单向**：L4 只能使机器更安全（通过 / 闩锁回退 / 记录移交），不得发布速度指令。

## 功能面（L0–L6）

| 层 | 名称 | 需求 |
|----|------|------|
| L0 | 本体 | 规划、控制、感知、任务二进制保持原样。 |
| L1 | 旁路捕获 | 滚动缓冲、触发、episode、MCAP、崩溃可 `RecoverMcap`。 |
| L2 | 因果 | 事件图；根因卡片；可选 re-roll 假设。 |
| L3 | 归因 | BT 失败定位到节点名与状态转移。 |
| L4 | 安全证据 | SHA-256 哈希链；独立 `safety` 子命令闩 `SafetyLatch`。 |
| L5 | 度量 / 门禁 | 机队 `metrics/episodes.jsonl` 上的 MPS / MPD、尾部延迟、`PublishGate`；`gate` CLI 读 `gate/latest.json`。 |
| L6 | 回放裁决 | 事实 / 干预回放；`REPLAY_PROFILE`：`open_loop` / `mid_stack` / `sensor_in`。 |

## 触发与优先级

| 触发 | `TriggerKind` | 默认优先级倾向 |
|------|---------------|----------------|
| 急停闩锁 | `TRIGGER_ESTOP` | P0 |
| 任务失败（BT 或 `*/feedback` FAILED / 定位质量低于阈值） | `TRIGGER_TASK_FAILURE` | P0 |
| 人工接管 | `TRIGGER_TAKEOVER` | P1 |
| BT recover、路径偏离、超时、代价尖峰 | 其余 | P2 或按分类器 |
| 成功 episode | — | P3，分层抽样 |

P0 立即上传；P1 上传但不当专家演示；P2 / P3 按 `p2_sample_rate` / `p3_sample_rate` 抽样。四元组不齐则无论优先级均写入 `diagnostics/`。
