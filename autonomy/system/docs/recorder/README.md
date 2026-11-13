# 数据闭环证据链

`autonomy.recorder` 是机上**旁路**进程：订阅 Autolink，将通道字节写入 MCAP，并在 episode 边界生成可裁决的 sidecar。不修改 planning / control / perception，不加入控制图。

短备忘：[`../../recorder/README.md`](../../recorder/README.md)。L4 闩锁：[`../safety/README.md`](../safety/README.md)。

## 阅读顺序

| 顺序 | 文档 | 内容 |
|------|------|------|
| 1 | [01 需求](01_requirements.md) | 目标、非目标、约束、L0–L6 功能面 |
| 2 | [02 方案](02_design.md) | 容器/载荷、四元组、双时钟、P0–P3、episode |
| 3 | [03 架构](03_architecture.md) | 进程拓扑、模块分层、与 monitor / Bridge / autoviz |
| 4 | [04 数据流](04_dataflow.md) | 采集 → 落盘 → pipeline → 回放 / 上传 |
| 5 | [05 使用](05_usage.md) | 构建、配置、launch、CLI、autoviz、产物 |

## 源码树对照

```text
autonomy/system/recorder/     # 实现
  proto/evidence.proto        # DualClock / VersionTuple / sidecar
  proto/recorder_options.proto
autonomy/system/conf/recorder.pb.txt
autonomy/system/launch/recorder.launch
autonomy/system/launch/recorder_full.launch
autonomy/system/launch/safety.launch
autoviz/.../ui/evidence/      # sidecar JSON 分诊 UI（不链 autonomy_system）
```

## 不变量

1. **旁路**：`RawMessage` + BEST_EFFORT + KEEP_LAST（高频 depth≈32，不是 1）；回调只拷贝预序列化字节。
2. **准入**：`policy_version` / `env_version` / `git_sha` / `seed` 任一缺失 → `diagnostics/`，不进训练。`policy_version` 可从 `AUTONOMY_SOFTWARE_VERSION` 回退。
3. **安全**：L4 不发布 `/cmd_vel`；超限只置 `SafetyLatch`，由 `autonomy.monitor` MRM 停机。
4. **观察**：无 `evidence_node_id` 的 observation 不得进入 L2。
