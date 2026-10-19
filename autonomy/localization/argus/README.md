# Argus

ROS-free port of **FAST-LIVO2** (`atlas/livo`) + **鲁棒定位感知层**（退化 / 动态 / 门控 / 回环 / 跨会话 / 自清洁）。

算法核**无** ROS/catkin、vikit、Sophus；**无**反向依赖 `atlas/livo`。

**生产完备清单：** [`docs/12_production.md`](docs/12_production.md)

## 文档

完整方案见 **[docs/00_guide.md](docs/00_guide.md)**（索引：[docs/README.md](docs/README.md)）。

**不可协商：** ESIKF 单线程串行；重定位 BnB 专用池不进实时链路；配置驱动 LIVO/LIO/LO；无 ROS；**禁止**无 LiDAR 的 ONLY_VIO 上线。

## Layout

| Path | Role |
|------|------|
| `argus/common/` | Logger、YAML、时间、`ThreadSafeQueue`、SO(3)、`math/transform`、SE(3)、camera |
| `argus/sensor/` | IMU / Image / PointCloud（库内类型，无数据集 I/O） |
| `argus/frontend/` | Preprocess、IMU、`SensorSync`、DynamicFilter |
| `argus/map/` | Voxel / Visual / MapSession / 自清洁 / IkdTree |
| `argus/mapping/` | MapBuilder 包 v3 |
| `argus/backend/` | `EsikfStateFilter` + `PoseGraph`（SE(3) GN） |
| `argus/fusion/` | ConfidenceGating / 各向异性 / IntensityAssist |
| `argus/perception/` | DegeneracyDetector / SensorHealthMonitor |
| `argus/odometry/` | `LivoOdometry::Process` + `ProcessStatus` |
| `argus/relocalization/` | `bbs/` · `scan_context/` · `loop_closure/`；门面 `manager` / `thread_pool` |
| `config/` | `livo_esikf.yaml` + `calibration/*` + `robustness/*` |
| `apps/` | `offline_*` / `slice|verify_kitti` / `verify_oxford_datasets` / `livo_esikf_node` |
| `apps/datasets/` | KITTI / Oxford Spires / Newer College 解析（不进 libargus） |
| `scripts/` | `download_` / `run_` / `plot_` / `viz_open3d_`（见 `scripts/README.md`） |
| `testdata/` | 真实数据集切片说明（见 `testdata/README.md`） |
| `test/` | 单元测试；子集进 `tools/argus/run_stub_tests.sh` |
| `docs/` | `00`–`12` 编号文档（见 `docs/00_guide.md`） |

CMake：GLOB → **单库** `libargus`。可选 ONNX：`ARGUS_USE_ONNXRUNTIME`。

## Real KITTI（C++ 解析）

```bash
# 从官方 odometry seq 拷贝前几帧（非合成）
slice_kitti --src /data/kitti/sequences/00 --dst testdata/kitti_real --frames 3
verify_kitti testdata/kitti_real
offline_kitti --kitti testdata/kitti_real --filter-only

# 或直接指向完整序列
export ARGUS_KITTI_ROOT=/data/kitti/sequences/00
verify_kitti
```

## Oxford Spires / Newer College（C++ 解析）

```bash
# Oxford Spires: sequence root 或 …/raw（lidar-clouds + cam0..2）
verify_oxford_datasets --spires /data/oxford_spires/sequences/2024-03-12-keble-college-01
# Newer College: EuRoC 式 raw（PCD 目录 + cam0/infra1）
verify_oxford_datasets --ncd /data/newer_college/01_short_experiment --camera cam0

export ARGUS_SPIRES_ROOT=/data/oxford_spires/sequences/...
export ARGUS_NCD_ROOT=/data/newer_college/...
```

### 离线评测脚本（`scripts/`，命名见 `scripts/README.md`）

```bash
# 1) 下载最小子集（默认 ~/Downloads/oxford_spires_dataset）
./scripts/download_spires_subset.sh

# 2) 单模式 + ATE 图（第 2 参 max_frames，0=全部）
./scripts/run_spires_eval.sh livo 0

# 3) LIVO / LIO / LO 同图对比 → …/argus_eval/modes/modes_compare.png
./scripts/run_spires_modes.sh 0

# 4) Open3D 全流程（需 open3d + build_spires/offline_spires_livo）
./scripts/run_spires_open3d.sh 2024-07-09-new-college-01 livo,lio,lo
SKIP_RUN=1 ./scripts/run_spires_open3d.sh 2024-07-09-new-college-01

# 5) 多序列下载 + 批量评测
./scripts/download_spires_sequences.sh \
  2024-07-09-new-college-02 \
  2024-03-12-keble-college-03
./scripts/run_spires_batch.sh 0 \
  2024-07-09-new-college-02 \
  2024-03-12-keble-college-03

# 可视化 / 出图
python3 scripts/viz_open3d_traj.py --seq-dir .../new-college-01 --modes lio,lo --map .../lio/map/map.xyz
python3 scripts/viz_open3d_mapping.py --seq-dir .../new-college-01 --mode lio --speed 1
python3 scripts/plot_traj_ate.py --gt gt.tum --est estimate.tum --out ate.png
```

配色：GT 黑 / LIVO 红 / LIO 蓝 / LO 紫；轨迹经 Umeyama 对齐到 VILENS GT。

## Build / Test

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=<path-with-automsgs>
cmake --build build
tools/argus/run_stub_tests.sh
offline_livo --synth config/livo_esikf.yaml
```

## Host API

```cpp
argus::common::InitLogging(argv[0]);
LivoOdometry odom(cfg);
OdometryOutput out;
if (odom.Process(&out) && !out.soft_skip) { /* out.state */ }
```

## Dependencies

Eigen3、OpenCV、glog、automsgs；OpenMP / ONNX Runtime 可选。无 ROS / vikit。
