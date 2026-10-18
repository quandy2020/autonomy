# 测试与性能

## 1. 单元测试（`ARGUS_BUILD_TESTS=ON`）

| 用例 | 覆盖 |
|------|------|
| `test_degeneracy_detector` | FIM 特征值与阈值 |
| `test_dynamic_filter` | 时空法向（stub CI） |
| `test_semantic_dynamic_filter` | 语义投影 + SetKeepMask（stub CI） |
| `test_confidence_gating` | 权重与跳过（stub CI） |
| `test_intensity_assist` | 强度辅助（stub CI） |
| `test_loop_closure` | Scan Context / 共识 / PoseGraph |
| `test_cross_session` | 链式回环与锚定 |
| `test_map_cleaning` | 置信度 prune |
| `test_load_livo_config` | YAML 冒烟 |
| `test_relocalization` | 队列 / BnB / 自动触发 |
| `test_pose_eval` | TUM ATE + LoadPath |
| `test_ikd_tree` | 增量 kNN |
| `test_map_builder` | 落盘 / prior / visual |
| `test_production_e2e` | 生产路径冒烟（若编入） |

真实 KITTI：`verify_kitti`（`ARGUS_KITTI_ROOT` 或 `testdata/kitti_real`）。

Stub CI：`tools/argus/run_stub_tests.sh`。

## 2. 应用级

| 程序 | 用途 |
|------|------|
| `offline_livo --synth` | 无 bag 合成 LO + 地图包 |
| `offline_kitti` | 真实 KITTI 过滤 / Process |
| `verify_oxford_datasets` | Spires / NCD 解析 |
| `eval_pose_traj` | 离线 ATE |
| `scripts/run_spires_*` | 全序列评测与可视化 |

## 3. 场景期望

| 场景 | 期望 |
|------|------|
| 隧道 / 长廊 | 不发散；弱轴方差升高 |
| 人群动态 | 残差不被行人主导 |
| 长距离环路 | 回环后漂移突降 |
| 跨会话重访 | 先验 + BnB / 链式回环 |

## 4. 性能指标与预算

| 指标 | 说明 |
|------|------|
| ATE / RPE | 有真值序列 |
| ESIKF update p50/p99 | 单次更新耗时 |
| 门控跳过率 | 模态被跳过比例 |
| 回环延迟 | 关键帧 → 因子注入 |
| BnB 占用 | 池队列深度、取消率 |

初值预算（可调）：

- ESIKF update p99 < 模态周期的约 80%（如 LiDAR 100 ms → 80 ms）
- 回环线程不挤占估计线程
- 自清洁单次 prune < 5 ms（或分片）
- 热路径禁 `cout` / 每帧 VIO 时序表；Occupancy Builder 单线程
- `LioFrameBuffers`：Livo 持有并 Bind；H/meas/残差 scratch / 点云 capacity 跨帧复用；VIO `Frame::Reset` + `copyTo`

## 5. 回归

合并前至少：`tools/argus/run_stub_tests.sh` + `offline_livo --synth`。  
全量数据集 bag 基准另开 CI artifact；本仓库以单测覆盖机制正确性。
