# Autoloop 数据闭环

端云飞轮：机上旁路采集 → 云端入库 / 转换 / 治理 / 训练 → 模型回灌机上。不改写 `autonomy` 规划 / 控制 / 感知，不插入 `/cmd_vel`。

机上证据 overlay 由 [`autonomy.recorder`](../../autonomy/system/docs/recorder/README.md) 实现（L0–L6）；Autoloop 负责把证据送进云、变成可训练数据、再原子回灌。

## 能力

| 能力 | 要回答的问题 | 文档 |
|------|----------------|------|
| 采集 | 失败窗口是否落盘且不背压控制？ | [data_loop](data_loop.md)、[trigger_rules](trigger_rules.md) |
| 可回溯 | 这次跑的是哪版 policy / 地图 / git / seed？ | [evidence_chain](evidence_chain.md) |
| 可追查 | 根因落在定位、规划、执行还是接管？ | [investigation](investigation.md) |
| 可复现 | SIL / HIL / 回归能否按同一四元组重放？ | [replay](replay.md) |
| 可可视化 | 时间轴、因果、机队仪表能否对齐同一 `episode_id`？ | [visualization](visualization.md) |
| 可治理 | 成败如何分流、未准入如何隔离？ | [data_loop](data_loop.md) §治理 |
| 可回灌 | 新模型如何原子切换且可回滚？ | [publishing](publishing.md) |
| 安全单向 | 采集 / 闩锁是否可能改写速度指令？ | [safety_domain](safety_domain.md) |

## 阅读顺序

| 顺序 | 文档 |
|------|------|
| 1 | [architecture](architecture.md) 端云拓扑与职责切分 |
| 2 | [data_loop](data_loop.md) 飞轮逐步：采集 → 上传 → 入库 → 训练 → 回灌 |
| 3 | [evidence_chain](evidence_chain.md) 四元组、双时钟、哈希、trace |
| 4 | [investigation](investigation.md) 诊断队列与根因 |
| 5 | [replay](replay.md) SIL / HIL / 回归 |
| 6 | [visualization](visualization.md) 分诊 UI 与仪表 |
| 7 | [publishing](publishing.md) 门禁与 `model_sync` |
| 8 | schema：[event](event_schema.md) / [log](log_schema.md) / [conversion](conversion_schema.md) / [naming](naming_conventions.md) |

## 不变量

1. **旁路**：`RawMessage` + BEST_EFFORT + KEEP_LAST；采集不发布控制话题。
2. **准入**：`policy_version` / `env_version` / `git_sha` / `seed` 任一空 → `admitted=false`，进诊断，不进训练 / 正式回放 / 发布。
3. **安全**：L4 只闩锁。`AUTOLOOP_LATCH_ONLY=1`。
4. **身份**：所有产物用 `robot_id` + `episode_id` + `trace_id` + `content_sha256` 串联。
5. **因果边界**：在线因果是时间窗 + 通道共现，不是物理力传递；物理复现走 SIL/HIL。
