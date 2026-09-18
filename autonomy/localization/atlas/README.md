# Atlas — 只要一套 multimodal SLAM

**唯一系统：** `atlas::system` + `sensor::SensorSuite` + `Pipeline::AttachSystem`。  
模态 `vo|vio|lo|lio|livo|wio|lwio|lvwio` = **同一套系统上的传感器/残差掩码**，不是多套可执行体。

## 架构总览

```mermaid
flowchart TB
  YAML["RuntimeConfig / profiles/*.yaml"]
  PIPE[Pipeline]
  SYS["ONE AtlasSystem"]
  YAML --> PIPE
  PIPE --> Suite[SensorSuite]
  PIPE --> SYS

  subgraph sensor ["sensor/ — 唯一测量入口"]
    CamB[CameraBridge]
    ImuB[ImuBridge]
    LidB[LidarBridge]
    OdomB[OdomBridge]
    Cam[CameraSensor]
    Imu[ImuSensor]
    Lid["LidarSensor + lightning preprocess/obs"]
    Odom[OdomSensor]
    CamB --> Cam
    ImuB --> Imu
    LidB --> Lid
    OdomB --> Odom
  end

  Suite --> sensor

  subgraph modules ["四大模块 — 同一 State"]
    FE["Frontend<br/>Tracking / LocalEstimator"]
    MP["Mapping<br/>LocalMapping / LocalJointBA / MapIncremental"]
    BE["Backend<br/>LoopClosing / GlobalJointBA"]
    RE["Relocalization<br/>Relocalizer sync in Tracking"]
  end

  SYS --> FE
  SYS --> MP
  SYS --> BE
  SYS --> RE
  SYS --> Pool["one ThreadPool"]

  Cam --> FE
  Imu --> FE
  Lid --> FE
  Odom --> FE
  Lid -.->|"IVox borrow"| MP
  Imu --> MP
  Lid --> MP
  Odom --> MP

  FE --> State["data/ Map + State"]
  MP --> State
  BE --> State
  RE --> State

  State --> Out["one T_world_base"]
  Out --> Viz[VizBridge]
  Out --> Side["io/ DenseMap · G2P5"]

  Pool -->|"Schedule Local BA"| MP
  Pool -->|"Schedule Global BA"| BE
  BE -->|"ScheduleAfter"| Side
```

## 数据与估计流

```mermaid
flowchart LR
  subgraph meas [测量]
    M1[IMU]
    M2[Camera]
    M3[Lidar]
    M4[Odom]
  end

  subgraph fe [Frontend 同步]
    Track[Tracking]
    LE[LocalEstimator ESKF]
  end

  subgraph est ["estimate/ ResidualMask"]
    Rv[vision]
    Ri[imu]
    Rl[lidar]
    Ro[odom]
  end

  subgraph opt [求解策略 — 同一 State]
    VIO["VO/VIO/LIVO：g2o staged Joint BA"]
    LIO["LO/LIO/WIO：LocalEstimator"]
  end

  M2 --> Track
  M1 --> Track
  M1 --> LE
  M3 --> LE
  M4 --> LE
  Track --> VIO
  LE --> LIO
  Rv --> VIO
  Ri --> VIO
  Rl --> VIO
  Ro --> VIO
  Mask[RuntimeConfig.residuals] --> est
```

## 目录树（实际）

```text
atlas/
├── system.* / pipeline.* / runtime_config.* / config.*
├── sensor/
│   ├── imu/          # ImuSensor · ImuBridge · buffer/preintegrator
│   ├── camera/       # CameraSensor · CameraBridge · 相机模型
│   ├── lidar/        # LidarSensor · LidarBridge · preprocess · obs_model
│   └── odom/         # OdomSensor · OdomBridge
├── frontend/         # Tracking · LocalEstimator · initializer/ · eskf/ · lio/ · lidar_loc/
│   ├── feature/ · match/ · solve/ · plp/ · initialize/
│   ├── eskf/         # frontend::Eskf (+ AndersonAcceleration)
│   ├── lio/          # MeasureGroup · ImuProcess · LidarImuSync
│   └── lidar_loc/    # LidarLocator · PoseExtrapolator（先验图定位骨架）
├── mapping/          # LocalMapping · LocalJointBA · MapIncremental · TiledMap · ivox/ · lidar_keyframe
│   └── ivox/         # mapping::IVox (+ hilbert.hpp)
├── backend/          # LoopClosing · LoopDetector · LidarLoopDetector · LidarPoseGraph · GlobalJointBA
├── relocalization/   # Relocalizer
├── estimate/         # ResidualMask · residual_{vision,imu,lidar,odom}
├── data/             # frame · keyframe · map_database · BoW
├── io/               # DenseMap · G2P5 · occupancy/cloud/octree…（计划中的 map/）
├── optimize/         # g2o 实现（冻结，经 estimate/ 暴露）
├── util/             # modality · schedule · publishers · converter…（计划中的 common/）
└── viz_bridge.*
```

| 路径 | 说明 |
|------|------|
| `sensor/` | 统一测量入口；**全部 ROS IO 经 `*Bridge`** |
| `frontend/` | Tracking + LocalEstimator + `eskf/` + feature/match/solve/plp/initialize |
| `mapping/` | LocalMapping + LocalJointBA + **MapIncremental** + **TiledMap** + `ivox/`（live IVox 所有者） |
| `backend/` | LoopClosing + GlobalJointBA |
| `relocalization/` | Relocalizer（BoW **同步**于 Tracking） |
| `estimate/` | ResidualMask + residual_* |
| `data/` | 唯一稀疏图 State |
| `io/` | DenseMap / G2P5 等侧路（显示/导航，不参与定位） |
| `optimize/` | g2o，**冻结不膨胀** |
| `util/` | schedule / modality / publishers |

## Canonical 类名

| §2b | 主类名（旧名兼容别名） |
|-----|-----------|
| `frontend/tracking.*` | `Tracking`（`using tracking_module = Tracking`） |
| `mapping/local_mapping.*` | `LocalMapping`（`using mapping_module = LocalMapping`） |
| `backend/loop_closing.*` | `LoopClosing`（`using global_optimization_module = LoopClosing`） |
| `frontend/eskf/` | `frontend::Eskf`（可选 Anderson AA + dx clip） |
| `frontend/lidar_loc/` | `frontend::LidarLocator` · `PoseExtrapolator` |
| `mapping/ivox/` | `mapping::IVox`（Morton / Hilbert key） |
| `sensor/lidar/.../preprocess` | `sensor::Preprocess` |
| `sensor/lidar/obs_model` | `sensor::ObsModel` |
| `mapping/local_joint_ba.hpp` | `mapping::LocalJointBA` |
| `backend/global_joint_ba.*` | `backend::GlobalJointBA` |
| `mapping/map_incremental.hpp` | `mapping::MapIncremental`（`IntegrateScan` 插入；可选 `set_tiled_map`） |
| `mapping/tiled_map.*` | `mapping::TiledMap`（chunk key / PCD+`index.yaml` Save·Load / `LoadOnPose`） |
| `relocalization/relocalizer.*` | `relocalization::Relocalizer` |
| `estimate/residual_*.hpp` | 共享残差 + `ResidualMask` |

## 模态与残差掩码

```mermaid
flowchart TB
  P[profiles/*.yaml] --> F[ModalityFlags]
  P --> R[ResidualMask]
  F --> S[SensorSuite 启停]
  R --> LJ[LocalJointBA]
  R --> GJ[GlobalJointBA]
  R --> LE[LocalEstimator 策略]
```

| Profile | 传感器 | 位姿权威 | 说明 |
|---------|--------|----------|------|
| **vo** | Cam | Tracking | 纯视觉 |
| **vio** | Cam+IMU | Tracking + IMU | 视觉惯性 |
| **lo** | Lidar | LocalEstimator | 雷达 ESKF update |
| **lio** | Lidar+IMU | LocalEstimator | PredictImu + UpdateLidar |
| **wio** | Odom+IMU | LocalEstimator | PredictImu + UpdateOdom |
| **lwio** | Lidar+Odom+IMU | LocalEstimator | 三源进同一 ESKF 路径 |
| **livo** | Cam+Lidar+IMU | Tracking + staged JointBA | mask 驱动 Local/Global |
| **lvwio** | 全开 | Tracking + JointBA + odom | 同上 + odom residual |

## Lidar LIO 进度（toward lightning-lm，无第二套 SLAM）

| 项 | 状态 | 说明 |
|----|------|------|
| **P0.1** MapIncremental owns insert | ✅ | `IntegrateScan`；`FeedWithPose` 只建残差；`LidarBridge::SetMapIncremental` |
| **Deskew** IMU 轨迹去畸变 | ✅ | `BuildImuPoses` + `UndistortByImuTrajectory`；无点时均匀 `t_rel` 近似；`<2` poses 回退 constant-ω |
| **Calib** 相机畸变 + 外参 | ✅ | `util/calibration`：`CalibrationBundle` / `undistort.hpp`；profile `calibration_path` |
| **IEKF** 解析 H | ✅ | `UpdateLidar` 迭代（`max_iekf_iter=4`）点面 H；信息形式 6×6 + Joseph；`PredictImu` Φ≈I+Fdt |
| **IMUInit** 重力 + bg | ✅ | `ImuProcess::TryImuInit`（~20 帧均值）；`Eskf::State::gravity`；`LidarBridge` 未就绪前跳过 deskew / UpdateLidar |
| **ESKF degeneracy** | ✅ | `UpdateLidar` 对 HTH 特征值投影不可观 DOF + `degeneracy_cov_inflation`；`predict_cov_inflation`；可选 **Anderson AA**（`enable_anderson`，默认 off）+ dx clip |
| **Lidar loop** | ✅ | PCL 多分辨率 NDT + `LidarPoseGraph`；Optimize 后同步 KF `T_wb` → `LocalEstimator::Reset`（统一 State）；LIVO 另写 `map_publisher`；`LoopClosedFn` → DenseMap `RequestRebuild`；**不**并 vision LoopClosing / GlobalBA；`use_lidar_loop` 默认 false |
| **G2P5** | ✅ | `io/g2p5` 全量对齐（显示/导航侧路）；`LidarBridge::SetG2P5` + 回环 `RedrawGlobalMap`；`maps.g2p5` |
| **Preprocess** | ✅ | 扁平 `sensor/lidar/preprocess`；`LidarModel` + `t`/`time`/`offset_time`；无时间时可选 `ring` 合成 |
| **IVox / TiledMap** | ✅ | Morton / **Hilbert** key 可选；面邻域 + capacity stats；`tiled_map` PCD+`index.yaml` Save/Load + `LoadOnPose`；`maps.tiled` |
| **LidarLocator** | ✅ | `frontend/lidar_loc/` 骨架：先验 TiledMap + PCL NDT；`PoseExtrapolator`；`enable_lidar_loc=false` |

Deferred（相对 lightning-lm，两边均无 Scan Context）：
- 产品层：pclomp NDT-OMP 全量 LocSystem、vision GlobalBA 与雷达回环 merge、原生 Livox CustomMsg / RoboSense 驱动包；G2P5 OccupancyGrid 话题 Writer（现有 `ToROS`/`ToCV`）；TiledMap **动静态分层**（现单层 PCD + optional occupancy）
- 说明：lightning 外参亦为固定；“外参入状态”两边都未做

## Lightning-lm `core/` ↔ Atlas

| Lightning | Atlas | 状态 |
|-----------|-------|------|
| `core/lio` | `frontend/{eskf,lio}` · `sensor/lidar/{preprocess,obs_model}` · `LocalEstimator` | 主链 partial（AA/clip ✅） |
| `core/ivox3d` | `mapping/ivox`（Morton/Hilbert）· `MapIncremental` | partial（无完整 PHC 节点） |
| `core/loop_closing` | `LidarLoopDetector` + `LidarPoseGraph`（统一经 LocalEstimator State + KF pose sync）；视觉 `LoopClosing` | partial（vision GlobalBA merge 仍 deferred） |
| `core/maps` | `mapping/tiled_map` · `io/{cloud_map,dense_map_builder}` | partial（PCD+index 已接；无 dyn 层） |
| `core/system` | `system` · `pipeline` · `runtime_config` · ThreadPool | done（单系统） |
| `core/miao` | `optimize/*`（g2o）· `graph_optimizer` · `lidar_pose_graph` | done（不引入 miao） |
| `core/g2p5` | `io/g2p5` | done（显示/导航侧路；`G2P5` + `G2P5Projector` facade） |
| `core/localization` | `frontend/lidar_loc`（LidarLocator + PoseExtrapolator 骨架） | skeleton（非第二套 SLAM） |

## 约定

- 无双系统 / PoseFusion / 独立 Lightning Start。
- 残差由 Pipeline `ResidualMask::FromRuntime` 注入 Local/Global Joint BA。
- JointBA：同 mask + State；单次 `optimize()` 可分阶段（vision/imu → lidar/odom refine）。
- ThreadPool：`Schedule` / `ScheduleAfter`（回环 BA 后 DenseMap `RequestRebuild`）。
- Reloc 保持 Frontend 同步（BoW 暂不 Schedule）。

## Runtime profiles

`conf/atlas/profiles/`：`vo` `vio` `lo` `lio` `livo` `wio` `lwio` `lvwio`。

新代码请 include §2b 路径与 `Tracking` / `LocalMapping` / `LoopClosing` 别名。
