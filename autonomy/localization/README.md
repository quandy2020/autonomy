# Localization

统一定位入口：`LocalizationServer` 按 `--localization_mode` 选择后端，进程内**只跑一套**位姿权威。

| 后端 | 适用 | 位姿权威 |
|------|------|----------|
| **Cartographer** | 2D/3D 激光 SLAM（默认） | Cartographer trajectory |
| **Atlas** | 多模态 SLAM（视觉 / 雷达 / 轮速 / IMU） | 同一 `atlas::system` + 模态掩码 |

Atlas **不是**第二套 Lightning 进程：lightning-lm 能力（deskew、IEKF、IVox、雷达回环、G2P5、TiledMap）合入 `atlas/`，见 [atlas/README.md](atlas/README.md)。

## 目录

```text
localization/
├── localization_main.cpp      # 可执行入口
├── localization_server.*      # Cartographer | Atlas 门面
├── atlas/                     # 多模态 SLAM（单系统）
├── cartographer/              # 激光图优化 SLAM
└── conf/
    ├── cartographer/          # *.lua
    └── atlas/
        ├── profiles/          # vo|vio|lo|lio|… 运行时 profile
        ├── lightning/         # 雷达 preprocess / loop / ivox
        ├── calibration/       # 内外参
        └── autosim_*.yaml     # 视觉系统 YAML
```

## Atlas 模态一览

| Mode | 传感器 | 位姿路径 |
|------|--------|----------|
| `vo` / `vio` | Cam〔+IMU〕 | Tracking〔+IMU〕 |
| `lo` / `lio` | Lidar〔+IMU〕 | LocalEstimator（ESKF） |
| `wio` / `lwio` | Odom+IMU〔+Lidar〕 | LocalEstimator |
| `livo` / `lvwio` | Cam+Lidar+IMU〔+Odom〕 | Tracking + JointBA |
| `atlas` | 同 `vio`，需配合 profile | 由 profile 覆盖 |

Profile：`conf/atlas/profiles/<name>.yaml`（传感器、残差、`lidar_config_path`、`maps.*`、`calibration_path`）。另有产品档 `lo_loc.yaml`（先验图 NDT 定位）、`lio_full.yaml`（`use_lidar_loop: true` + IMU 高周波姿）。

## 一键运行 LIO（autosim 验证）

话题已对齐：`/imu`、`/points`，**无需 remap**。场景需已有
`/workspace/autonomy/src/17DRP5sb8fy/17DRP5sb8fy.glb`（与 autosim 默认一致）。

**终端 1 — autosim（开 3D 雷达）**

```bash
cd /workspace/autonomy
CONFIG=src/autonomy/autosim/config/lio.yaml src/autonomy/autosim/scripts/run.sh sim
# 另开交互 TTY 遥控：
# src/autonomy/autosim/scripts/run.sh teleop
```

**终端 2 — Atlas LIO**

```bash
cd /workspace/autonomy/src/autonomy
export PATH=/workspace/autonomy/build/autonomy/bin:$PATH
export AUTOLINK_LAUNCH_PATH=/workspace/autonomy/src/autonomy/autonomy/localization/launch

autolink launch start localization_lio.launch
```

停止：

```bash
autolink launch stop localization_lio.launch
```

对应：`autosim/config/lio.yaml`（`lidar_3d.enabled=true`）+
[launch/localization_lio.launch](launch/localization_lio.launch)。

- OccupancyGrid：`/atlas/g2p5/occupancy`
- 雷达回环：在 `conf/atlas/lightning/default.yaml` 设 `use_lidar_loop: true`，或用产品档 `profiles/lio_full.yaml` / `localization_lio_full.launch`
- LO/LIO 位姿：IMU 率 `PoseExtrapolator`（本地外推）→ `VizBridge`；**estimator PredictImu 仅 deskew**；`body_flu_pose`；可选 lightning `smooth_factor`
- 无视觉，不需要 `--atlas_config` / vocab

等价手写 LIO 命令（调试用）：

```bash
autonomy.localization \
  --localization_mode=lio \
  --atlas_runtime_profile=autonomy/localization/conf/atlas/profiles/lio.yaml \
  --atlas_imu_topic=/imu \
  --atlas_lidar_topic=/points \
  --atlas_lidar_imu_topic=/imu
```

## 使用指南

入口二进制：`autonomy.localization`（或包名 `localization`）。工作目录需能解析 `autonomy/localization/conf/...`。

### Cartographer（默认）

```bash
autonomy.localization \
  --localization_mode=cartographer \
  --configuration_directory=autonomy/localization/conf/cartographer \
  --configuration_basename=backpack_2d.lua
```

可选：`--load_state_filename=...pbstream`、`--save_state_filename=...`。

### Atlas — 先验图定位（`lo_loc`）

先填 `lo_loc.yaml` 的 `maps.tiled_path`（含 `index.yaml` + `chunk_*.pcd`）：

```bash
autonomy.localization \
  --localization_mode=lo \
  --atlas_runtime_profile=autonomy/localization/conf/atlas/profiles/lo_loc.yaml
```

主环 `LidarLocator::Align`（PCL NDT）+ `PoseExtrapolator`；有 dyn 层时对齐只用 static。

### Atlas — 视觉 / VIO

```bash
autonomy.localization \
  --localization_mode=vio \
  --atlas_runtime_profile=autonomy/localization/conf/atlas/profiles/vio.yaml \
  --atlas_config=autonomy/localization/conf/atlas/autosim_mono.yaml \
  --atlas_vocab=autonomy/localization/conf/atlas/orb_vocab.fbow \
  --atlas_rgb_topic=/camera/rgb/image_raw \
  --atlas_imu_topic=/imu
```

RGB-D / 仿真可换 `autosim_rgbd.yaml`。地图：`--atlas_map_load` / `--atlas_map_save`。

### Atlas — LIVO（视觉 + 雷达）

```bash
autonomy.localization \
  --localization_mode=livo \
  --atlas_runtime_profile=autonomy/localization/conf/atlas/profiles/livo.yaml \
  --atlas_config=autonomy/localization/conf/atlas/autosim_rgbd.yaml \
  --atlas_vocab=autonomy/localization/conf/atlas/orb_vocab.fbow
```

视觉 LoopClosing 与雷达 NDT 回环**独立**；雷达回环经 `LocalEstimator` / `map_publisher`，并轻踢 Tracking，**不**写入视觉 GlobalBA 边。

### 常用 Flag

| Flag | 含义 |
|------|------|
| `--localization_mode` | `cartographer` 或 `vo\|vio\|lo\|lio\|livo\|wio\|lwio\|lvwio\|atlas` |
| `--atlas_runtime_profile` | `conf/atlas/profiles/*.yaml`（模态、话题、`lidar_config_path`） |
| `--atlas_config` / `--atlas_vocab` | 视觉系统 YAML + ORB 词袋（含视觉时必填） |
| `--atlas_lidar_config` | 覆盖 profile 的 lightning YAML；空则用 profile `lidar_config_path` |
| `--atlas_imu_topic` / `--atlas_lidar_topic` / `--atlas_wheel_topic` | 传感器话题 |
| `--atlas_enable_lightning_upstream` | 打开 Lidar ObsModel 残差上游（默认 false） |
| `--atlas_map_load` / `--atlas_map_save` | Atlas 地图库 |

CLI 非空优先；`--atlas_lidar_config` 空时回落 profile。

## 配置速查

| 路径 | 用途 |
|------|------|
| `conf/atlas/profiles/*.yaml` | 模态、传感器、残差、`lidar_config_path`、`maps.g2p5(_topic|_save_path)` / `tiled(_path)`、`enable_lidar_loc` |
| `conf/atlas/profiles/lo_loc.yaml` | 先验 TiledMap NDT 定位产品档 |
| `conf/atlas/profiles/lio_full.yaml` | LIO 全产品档（`lightning/lio_full.yaml`，回环开） |
| `conf/atlas/lightning/default.yaml` | preprocess、`use_lidar_loop`（默认 false）、ivox、obs_model |
| `conf/atlas/lightning/lio_full.yaml` | 同 default，但 `use_lidar_loop: true` |
| `conf/atlas/calibration/default.yaml` | 相机 / IMU / 雷达内外参 |
| `conf/cartographer/*.lua` | Cartographer 参数 |

## 约定

- 进程内单一后端；Atlas 内单一 `system`，模态只改传感器与残差掩码。
- LO/LIO/WIO：无视觉 YAML，位姿在 `LocalEstimator`。
- 含视觉：必须提供 `--atlas_config` + vocab。
- 更深架构、LIO 能力表、lightning 对照 → [atlas/README.md](atlas/README.md)。
