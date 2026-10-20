# 回环与跨会话

时序见 [03_sequence.md §4](03_sequence.md)；数学见 [04_math.md §6–7](04_math.md)。

## 1. 目录

根路径：`argus/relocalization/loop_closure/`（命名空间仍为 `argus::loop_closure`）。

```text
relocalization/loop_closure/
├── types.hpp / manager.* / consensus.* / cross_session.*
└── scan_context/detector.*
```

| 路径 | 职责 |
|------|------|
| `manager.*` / `types.hpp` | 异步门面与检测器接口 |
| `consensus.*` / `cross_session.*` | 共识与跨会话 |
| `scan_context/` | Scan Context / SC++ |
| `PoseGraph` | `backend/pose_graph/` |
| `MapSession` | `map/map_session.*` |

## 2. 同会话回环

```text
LIO 成功 → TrySubmitKeyFrame
  → LoopClosureManager::SubmitKeyFrame（非阻塞）
  → [async] ScanContext Top-K → Consensus
       → TryAcceptLoops (Snapshot → Optimize → FPR → Restore|commit)
       → LoopCorrection(ΔT) + 可选 LTA EnqueueCorrectedCloud
估计线程 TryInjectLoopCorrection
  → 注入 ΔT；NotifyPoseJump；Drain LTA → MapBuilder::ikd().AddPoints
```

硬约束：`Detect` / `Optimize` / FPR **仅** worker；`loop_closure.async` 加载时强制 `true`。

位姿图求解优先级：GTSAM LM（`-DARGUS_USE_GTSAM=ON` → `ARGUS_HAS_GTSAM`）→ Ceres → 稠密 GN。FPR 使用批式重建图（非 iSAM2 增量），以便 Snapshot/Restore。

跨会话：`LinkAndOptimize` 走与同会话相同的 `TryAcceptLoops` 硬 FPR；拒绝时不提交 loop、不计跨会话成功。

Prior LTA：`LoadBuiltMap` → `MapBuilder::ExportLoopKeyFrames()`（`id=node_id_base+local_id`，world 云转 body）→ `RegisterPriorSession` 缓冲；跨会话成功后按 `lta.prior_neighbor_keyframes` 邻域入 `MapReloadWorker`。

## 3. 配置

YAML 键仍为 `loop_closure.*`（与目录名解耦）：

| 键 | 含义 |
|----|------|
| `enable` | 默认 false |
| `async` | 必须 true |
| `enable_pose_inject` | Optimize 后注入 ESIKF |
| `fpr.*` | 硬 FPR：`max_loop_residual` / `max_cost_rms` |
| `wait_second_loop` | 双回环再优化（结构相似场景） |
| `lta.*` | 历史/prior 点回灌（默认 off）；`prior_neighbor_keyframes`、`reload_prior_on_cross_session` |
| `scan_context.*` | 环扇 / lateral（SC++） |
| `consensus.*` / `cross_session.*` | 共识与跨会话 |

## 4. 测试

`test_pose_graph` / `test_map_reload` / `test_prior_lta_reload` / `test_loop_closure` / `test_cross_session`。

计划：[`plans/2026-10-19-argus-lta-fpr.md`](plans/2026-10-19-argus-lta-fpr.md)。
