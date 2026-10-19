# 用法

## 1. 宿主契约

```text
任意线程:  AddLidar / AddImu / AddImage
估计线程:  while (Process(&out)) { use out.state; check out.status; }
可选:      SaveBuiltMap / LoadBuiltMap / RequestRelocalization / eval_pose_traj
```

```cpp
#include "argus/odometry/livo_odometry.hpp"
#include "argus/odometry/load_livo_config.hpp"

auto loaded = argus::odometry::LoadLivoOdometryConfig("config/livo_esikf.yaml");
argus::odometry::LivoOdometry odom(loaded.config);
odom.Initialize();

odom.AddImu(imu);
odom.AddLidar(cloud, t);
odom.AddImage(img, t);

argus::odometry::OdometryOutput out;
while (odom.Process(&out)) {
  if (out.soft_skip) continue;
  // out.state: pose / vel / ...
  // out.status: ProcessStatus
}
odom.SaveBuiltMap(output_dir);
```

约束：

- `threading.backend_threads == 1`
- 重定位 / 回环 / 自清洁 **永不**在估计线程内跑 Optimize / Search
- Apps 启动调用 `argus::common::InitLogging`

## 2. 构建

```bash
# 全量（需 automsgs）；默认 ARGUS_BUILD_SHARED=ON → libargus.dylib
cmake -S autonomy/localization/argus -B build/argus \
  -DARGUS_BUILD_TESTS=ON -DARGUS_BUILD_APPS=ON \
  -DCMAKE_PREFIX_PATH=<path-with-automsgs>
cmake --build build/argus
ctest --test-dir build/argus

# 静态库
cmake -DARGUS_BUILD_SHARED=OFF ...

# Stub CI（无 automsgs）
tools/argus/run_stub_tests.sh

# 可选 ONNX
cmake -DARGUS_USE_ONNXRUNTIME=ON ...
```

产物示例：`libargus.dylib` / `build_spires/offline_spires_livo`（本地评测见 `scripts/_argus_env.sh`）。

## 3. Apps

| 程序 | 用途 |
|------|------|
| `offline_livo` | YAML 初始化；`--synth` 合成 LO + 地图包 |
| `offline_spires_livo` | Oxford Spires 离线 LIVO/LIO/LO |
| `offline_kitti` | KITTI → 过滤 / LO |
| `slice_kitti` / `verify_kitti` | 切片与解析校验 |
| `verify_oxford_datasets` | Spires / Newer College raw 冒烟 |
| `livo_esikf_node` | 进程入口冒烟 |
| `eval_pose_traj` | TUM ATE |
| `offline_livo` MapBuilder | `map.xyz` + `map_meta` |

```bash
offline_livo --synth config/livo_esikf.yaml
verify_oxford_datasets --spires /data/oxford_spires/sequences/<seq>
export ARGUS_SPIRES_ROOT=/data/oxford_spires/sequences/<seq>
```

## 4. 离线评测脚本

约定与一览：[`scripts/README.md`](../scripts/README.md)。

```bash
cd autonomy/localization/argus

# 下载默认序列
./scripts/download_spires_subset.sh

# 单模式 + ATE
./scripts/run_spires_eval.sh lio 0

# LIVO / LIO / LO 同图
./scripts/run_spires_modes.sh 0

# 离线 + ATE + Open3D
./scripts/run_spires_open3d.sh 2024-07-09-new-college-01 livo,lio,lo
SKIP_RUN=1 ./scripts/run_spires_open3d.sh 2024-07-09-new-college-01

# 多序列
./scripts/download_spires_sequences.sh SEQ2 SEQ3
./scripts/run_spires_batch.sh 0 SEQ2 SEQ3

# 可视化
python3 scripts/viz_open3d_traj.py --seq-dir .../SEQ --modes lio,lo --map .../lio/map/map.xyz
python3 scripts/viz_open3d_mapping.py --seq-dir .../SEQ --mode lio --speed 1
python3 scripts/plot_traj_ate.py --gt gt.tum --est estimate.tum --out ate.png
```

常用环境变量：`ARGUS_SPIRES_DEST`、`ARGUS_SPIRES_SEQ`、`ARGUS_SPIRES_BIN`、`ARGUS_LIVO_CONFIG`、`ARGUS_FORCE_RERUN`、`SKIP_RUN`。

配色：GT 黑 / LIVO 红 / LIO 蓝 / LO 紫；轨迹 Umeyama 对齐 VILENS GT。

## 5. 重定位先验

```cpp
// 推荐：占据场 + 全量 .xyz（SC++ place index 质量更好）
auto bundle = PriorMapLoader::LoadPathBundle(path, resolution);
odometry.relocalization_manager()->SetPriorMap(bundle);

// 大地图打开候选裁剪
// relocalization.place_recognition.enable: true
```

## 6. 配置入口

见 [09_config.md](09_config.md)。生产检查见 [12_production.md](12_production.md)。
