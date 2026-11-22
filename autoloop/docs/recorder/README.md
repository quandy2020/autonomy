# 数据闭环证据链

`autoloop.recorder` 是机上**旁路**进程：订阅 Autolink，将通道字节写入 MCAP，并在 episode 边界生成可裁决的 sidecar。不修改 planning / control / perception，不加入控制图。

短备忘：[`../../autoloop/robot/recorder/README.md`](../../autoloop/robot/recorder/README.md)。端云飞轮：[../README.md](../README.md)（编号 01–15）。L4：[11 安全域](../11_safety.md)。

## 阅读顺序

| 全局编号 | 文档 | 内容 |
|----------|------|------|
| 16 | [01 需求](01_requirements.md) | 目标、非目标、约束、L0–L6 功能面 |
| 17 | [02 方案](02_design.md) | 容器/载荷、四元组、双时钟、P0–P3、episode |
| 18 | [03 架构](03_architecture.md) | 进程拓扑、模块分层、与 monitor / Bridge / autoviz |
| 19 | [04 数据流](04_dataflow.md) | 采集 → 落盘 → pipeline → 回放 / 上传 |
| 20 | [05 使用](05_usage.md) | 构建、配置、launch、CLI、autoviz、产物 |

## 源码树对照

```text
robot/recorder/     # 实现
  proto/evidence.proto        # DualClock / VersionTuple / sidecar
  proto/recorder_options.proto
robot/config/data_agent/recorder.pb.txt
robot/bringup/launch/recorder.launch
robot/bringup/launch/recorder_full.launch
robot/bringup/launch/recorder_safety.launch
autoviz/.../ui/evidence/      # sidecar JSON 分诊 UI（不链 autonomy_system）
```

## 不变量

1. **旁路**：`RawMessage` + BEST_EFFORT + KEEP_LAST（高频 depth≈32，不是 1）；回调只拷贝预序列化字节。
2. **准入**：`policy_version` / `env_version` / `git_sha` / `seed` 任一缺失 → `diagnostics/`，不进训练。`policy_version` 可从 `AUTONOMY_SOFTWARE_VERSION` 回退。
3. **安全**：L4 不发布 `/cmd_vel`；超限只置 `SafetyLatch`，由 `autonomy.monitor` MRM 停机。
4. **观察**：无 `evidence_node_id` 的 observation 不得进入 L2。
