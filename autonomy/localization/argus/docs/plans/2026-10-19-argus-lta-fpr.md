# Argus LTA / FPR Implementation Plan

> **For agentic workers:** Implement task-by-task; keep ESIKF thread free of Optimize / map rebuild.

**Goal:** Port LTA-OM ideas (hard FPR + history-map reload + jump sync) into Argus without ROS.

**Architecture:** Async loop worker owns pose-graph FPR; optional `MapReloadWorker` feeds a bounded historical-point deque drained on the estimate thread. Pose-graph solver: **Ceres** by default (`ARGUS_HAS_CERES`); optional **GTSAM iSAM2** when `ARGUS_USE_GTSAM=ON`.

**Tech Stack:** C++17, Ceres, Eigen, (optional) GTSAM, existing Argus map/loop modules.

**Spec:** Conversation design + LTAOM local scan under `ros-perception/LTAOM`.

## Global Constraints

- Never run Optimize / FPR / full map rebuild on the ESIKF thread.
- No GTSAM system-source patches; optional wrapper only.
- LTA default **off** in YAML until validated.
- Preserve BnB reloc + SC++; LTA complements, does not replace.

## Tasks

- [x] S1 PoseGraph: Snapshot / Restore / TryAcceptLoops + Ceres Optimize
- [x] S2 FPR wired into LoopClosureManager + config
- [x] S3 MapReloadWorker + Drain on estimate path hooks
- [x] S4 YAML + docs + stub tests (`tools/argus/run_stub_tests.sh` green)
- [x] S5 GTSAM batch LM backend (`pose_graph_gtsam.cpp`, `-DARGUS_USE_GTSAM=ON`; Homebrew 无配方时跳过链接)
- [x] S6 Multisession prior chunks via MapReloadWorker (`ReloadPriorNeighborhood`)

---

### Solver note (S5)

GTSAM 路径为 **批式** `LevenbergMarquardtOptimizer`（每轮 Optimize 重建 `NonlinearFactorGraph`），与 FPR Snapshot/Restore 一致。增量 iSAM2 可后续再加，但不宜与硬回滚混用同一 Values 缓存。

### Prior LTA (S6)

`RegisterPriorSession(..., scan_context_keyframes)` 同时缓冲 body 云；跨会话 `LinkAndOptimize` 成功后，按 `lta.prior_neighbor_keyframes` 邻域 `WorldPose` 变换入队。

### Hardening (post S6)

- Package `keyframes/` → `ExportLoopKeyFrames`（修正 global id + world→body）
- 跨会话 `LinkAndOptimize` 对齐 `TryAcceptLoops` / FPR

### Eval hardening (v6–v8)

- [x] Split `fpr_reject` / `inject_restore` + geom fail histogram in loop stats
- [x] Temporal match-id consistency (`temporal.*`; fix `min_confirmations` floor)
- [x] Synthetic drift + `scripts/run_spires_loop_drift_ab.sh` for recovery A/B
- [x] `inject_clamp_only`: keep graph on large Δ; clear \(T_d\) without ESIKF yank
- [x] Drift YAML: `loop_closure_drift_eval.yaml` / `livo_esikf_loop_drift.yaml`
- [x] `revisit_signature` + pre-Optimize-only check; absorb \(T_d\) on clamp recovery
- [x] Drift A/B v10: ATE 3.93 → 3.23 m (−18%) with accepted=1
