# Architecture

Autoloop 把一次业务失败变成可归档、可检索、可复现、可训练、可回灌的闭环。机上只旁路；云上只消费证据，不远程改写控制图。

## 飞轮

```
 ┌─────────── 机上（robot）───────────┐         ┌─────────── 云端（cloud）───────────┐
 │  L0 autonomy 规划/控制/感知（只读）│         │  ingestion  哈希+四元组准入        │
 │           │ Autolink 通道          │         │  conversion MCAP→LeRobot          │
 │  recorder │ data_agent 旁路        │  P0–P3  │  governance 成败池 / 人审         │
 │  ring → trigger → MCAP+sidecar ───┼────────►│  datasets   训练 / 回归集          │
 │  safety_guard 闩锁（不写 cmd_vel） │  上传   │  training   产出 policy 工件       │
 │  telemetry / structured_log        │         │  publishing 门禁 → registry       │
 │           ▲                        │         │  diagnostics 索引 / 回放 / 回归   │
 │  model_sync 原子切换 current ──────┼─────────┤  observability 仪表 / 告警        │
 └────────────────────────────────────┘  回灌   └───────────────────────────────────┘
```

一次闭环最小闭环：**触发落盘 → 准入归档 → 转换/治理 → 训练或回归 → 发布 → 机上切换 → 下一 episode 四元组带上新 `policy_version`**。

## 职责切分

| 域 | 做什么 | 不做什么 |
|----|--------|----------|
| `autonomy` L0 | 规划、控制、感知、任务 | 不被 Autoloop 改源码、不插 `/cmd_vel` 转发 |
| `autonomy.recorder` | 机上证据 overlay（L1–L6 采集与 sidecar） | 不上云、不训练、不发模型 |
| `autoloop.robot.data_agent` | 与 recorder 对齐的旁路：ring / 触发 / MCAP / 上传 / 闩锁 | 不替代 recorder 的 L2 图构建；不发布控制 |
| `autoloop.robot.model_sync` | 拉模型、校验四元组、原子切 `current` | 不在任务中途热切 |
| `autoloop.cloud` | 入库、转换、治理、训练入口、发布、诊断、可视化数据面 | 不远程刹停机器人 |
| autoviz / dashboards | 只读 MCAP + sidecar + 指标 | 不写控制通道 |

`drivers` / `localization` / `navigation` / `perception` 在 Autoloop 树内是 **loop-runtime 接口**，供仿真回放与机上代理对接；家居导航本体仍在 `autonomy/`。

## 进程拓扑

```
机上:  autonomy.* ──Autolink──► data_agent ──► log/ + 上传队列
                 │                 │
                 └── safety_guard ─┘  共享闩锁文件
云端:  record_receiver → parser → dedup → convert → pools
       metadata_store / trace_index ←── 全部 sidecar
       model_publisher → registry → 机上 model_sync
```

采集进程与控制进程隔离。上传失败只积压队列，不阻塞 `OnSample`。

## 身份与产物

每个 episode 一组同 stem 文件：

```
ep-<utc_ns>-<kind>/
  snapshot.mcap      # 通道字节 + DualClock
  sidecar.pb         # VersionTuple、admitted、missing_fields、content_sha256
  sidecar.json       # 可视化 / 清单镜像
  events.jsonl       # 结构化事件（可选）
```

云端归档键：`robot_id / episode_id / content_sha256`。训练样本必须能从 LeRobot 行反查回该键。

## 分层

| 层 | Autoloop 落点 | 判定 |
|----|---------------|------|
| L0 本体 | 不修改 | 控制图不变 |
| L1 捕获 | `data_agent` + recorder | MCAP 可恢复 |
| L2 时间因果 | sidecar 事件 + 云 `trace_index` | 窗内共现，非物理因果 |
| L3 归因 | BT / 任务 feedback / 定位质量 | 节点名可检索 |
| L4 安全 | `safety_guard` + SHA-256 链 | 只闩不控 |
| L5 度量 | telemetry → `telemetry_gate` | MPS/MPD、接管率 |
| L6 回放 | `simulation/replay` + `regression_runner` | 四元组一致才算正式 |

## 失败隔离

| 状态 | 去向 |
|------|------|
| 四元组缺字段 | diagnostics，禁止训练 |
| 哈希不匹配 | 拒收，告警 `data_loss` |
| 上传中断 | 断点续传，P0 优先 |
| 安全链断裂 | 失败池 + `safety_event` 告警 |
| 门禁未过 | registry 不更新 `current` |
