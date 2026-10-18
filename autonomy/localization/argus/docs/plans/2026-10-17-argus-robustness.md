# Argus 鲁棒定位感知层 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在现有 ESIKF + Voxel/Visual Map + VIO 主路径上，按 P0→P6 补齐退化、动态过滤、门控、回环、跨会话与地图自清洁，且不破坏「ESIKF 串行 / 异步解耦 / 无 ROS」约束。

**Architecture:** 判断在前端或 `ConfidenceGating`；ESIKF 只消费加权残差；回环 / PoseGraph / 自清洁 / BnB 均异步或专用池。布局保持 `argus/<module>/` 同树。

**Tech Stack:** C++17、Eigen、OpenCV、glog、automsgs、可选 OpenMP / ONNX Runtime。

**Spec:** [../architecture.md](../architecture.md)、[../roadmap.md](../roadmap.md)

## Global Constraints

- `threading.backend_threads == 1`
- 重定位线程池不进入实时估计
- 新模块默认不进 CMake，直至单测通过再 `add_library`
- 配置驱动模态；禁止为 LIVO/LIO 复制滤波器实现
- 不依赖 ROS / vikit / Sophus

---

### Task P0.1：DegeneracyDetector 接口与单测

**Files:**
- Create: `argus/perception/degeneracy_detector.hpp` / `.cpp`
- Create: `test/test_degeneracy_detector.cpp`（待 CMake test 目标）
- Modify: `config/robustness/degeneracy.yaml`

- [x] 实现 `Analyze(fim)`：特征值、条件数、弱方向
- [x] 合成对角 FIM 单测（弱轴检出）
- [x] 文档示例与阈值一致

### Task P0.2：AnisotropicCovariance + 接入更新前

**Files:**
- Create: `argus/fusion/anisotropic_covariance.hpp` / `.cpp`
- Modify: `argus/backend/state_filter/esikf_state_filter.*`（或 odometry 编排处）

- [x] `Inflate(P, u, alpha)` rank-1 膨胀
- [x] 在 LiDAR 更新前可选调用；默认配置关闭直至 P2 门控统一
- [x] 单测：正交方向方差近似不变
- [x] `LivoOdometryConfig.enable_degeneracy` + `EsikfUpdateOptions` 接线
- [x] `Argus::perception` / fusion gating 编入 CMake

### Task P1：DynamicFilter

**Files:**
- Create: `argus/frontend/dynamic_filter_base.hpp`
- Create: `spatiotemporal_dynamic_filter.*`、`semantic_dynamic_filter.*`
- Modify: preprocess / VIO 入口应用 mask

- [x] Base + 时空（球面距离）最小实现
- [x] 语义路径可编译关闭（无 ONNX → pass-through）
- [x] SemanticDynamicFilter ONNX 实装（`ARGUS_HAS_ONNXRUNTIME`）+ `mask_file` 离线后端
- [x] 真实 KITTI：`apps/datasets/KittiDataset`（odometry/raw）+ `slice_kitti` / `verify_kitti`
- [x] `test_dynamic_filter`
- [x] `LivoOdometry` 下采样后应用 LiDAR mask

### Task P2：ConfidenceGating

**Files:**
- Create: `argus/fusion/confidence_gating.*`
- Create: `argus/perception/sensor_health_monitor.*`
- Modify: `LivoOdometry` / ESIKF 更新调用链

- [x] `Evaluate` 产出 `ObservationConfidence`（含 LiDAR 动态比例）
- [x] 跳过无效模态；权重缩放 H/HTz；权重下限
- [x] `test_confidence_gating`
- [x] `LivoOdometry` 健康度打点 + `enable_confidence_gating`

### Task P3：Scan Context + PoseGraph

**Files:**
- Create: `argus/loop_closure/scan_context_detector.*`
- Create: `argus/backend/pose_graph/*`
- Modify: 关键帧投递与异步 worker

- [x] 描述子 + Top-K 检索（`ScanContextDetector`）
- [x] PoseGraph 因子与异步 Optimize（xy+yaw GN）
- [x] 估计线程无 Optimize（仅 `SubmitKeyFrame`）
- [x] `Argus::loop_closure` + `test_loop_closure`

### Task P4：跨会话

**Files:**
- Create: `map_session.*`、`cross_session_linker.*`
- Docs: `cross_session.md` 已存在，补序列化字段

- [x] 会话加载 / 注册（`MapSession` + `RegisterPriorSession`）
- [x] 链式因子生成（`CrossSessionLinker::MakeChainFactors`）
- [x] 双会话离线用例（`test_cross_session`）

### Task P5：地图自清洁

**Files:**
- Create: `map_confidence.*`、`map_self_cleaning.*`
- Modify: voxel 插入路径写置信度

- [x] 置信度更新与低频 prune（`UpdateConfidence` + 异步 `Prune`）
- [x] 读写锁文档与实现一致（`VoxelMapManager::map_mutex_`）
- [x] `test_map_cleaning` + CMake

### Task P6：增强

- [x] 强度辅助约束（`IntensityAssist` 缓解退化协方差膨胀）
- [x] `MultiModalConsensus` 打分增强（pose consistency + visual_score）
- [x] 性能场景 1–3 机制就绪并文档化（[../performance.md](../performance.md)；全量 bag 待 offline app）

### 收尾

- [x] 通过单测的库已在 `CMakeLists.txt`（perception / map / fusion / backend / loop_closure / odometry）
- [x] 更新根 README 布局 / 依赖说明（去掉过时 Placeholders）
- [x] `apps/livo_esikf_node` / `offline_livo` 读取 `livo_esikf.yaml` 冒烟（`LoadLivoOdometryConfig`）
- [x] Relocalization `tracking_lost` 自动触发（streak / cooldown）
- [x] MapBuilder / VisualMap 地标落盘（`visual.xyz`，无 bag）
- [x] BnB `se3_yaw` + roll/pitch 微调；`EvaluateAgainstGroundTruth`
- [x] `prior_map_path` / `cross_session_load`；TUM `eval_pose_traj`
- [x] 回环 `LoopCorrection` ΔT 注入 ESIKF；`mapping.enable` → MapBuilder
- [x] `offline_livo --synth`；`tools/argus/run_stub_tests.sh` + GitHub workflow
- [x] VIO 收进 `LivoOdometry::Process`（共享 `VIOManager`；Atlas `handleVIO` 仅发布）
- [x] `camera/calib_file` + `vio.*` / `extrinsic.Rcl|Pcl` 配置加载
- [x] `test_vio_manager` 视觉冒烟（stub CI）
- [x] 地图包 v2：`map_meta` / `map.xyz` / `prior.xyz` / `visual.xyz` / `trajectory.tum` Save/Load
- [x] `LivoOdometry::LoadBuiltMap` 恢复地图并注入重定位先验
- [x] 场景退化默认 on + 强度辅助默认 on；stub 含 `test_map_builder` / `test_degeneracy_detector`
- [x] PoseGraph SE(3) Gauss-Newton；BnB `search_space: se3` 6-DoF 局部精化
- [x] 地图包 v3：原子写 + `map_xyz_fnv64` 校验
- [x] `ONLY_VIO` 传感器同步；`test_production_e2e` 生产冒烟
- [x] `ProcessStatus` / `soft_skip`；配置生产校验（禁 ONLY_VIO 上线）
- [x] `visual` 动态过滤接线；地图包 `keyframes/` → 跨会话 SC
- [x] Apps `InitLogging`；stub CI 扩覆盖；[`production.md`](../production.md)
