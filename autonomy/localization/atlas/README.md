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
│   ├── lidar/        # LidarSensor · LidarBridge · lightning/{preprocess,obs_model}
│   └── odom/         # OdomSensor · OdomBridge
├── frontend/         # Tracking · LocalEstimator · initializer/ · eskf/ · lio/
│   ├── feature/ · match/ · solve/ · plp/ · initialize/
│   ├── eskf/         # frontend::Eskf
│   └── lio/          # MeasureGroup · ImuProcess · LidarImuSync
├── mapping/          # LocalMapping · LocalJointBA · MapIncremental · ivox/ · lidar_keyframe
│   └── ivox/         # mapping::IVox
├── backend/          # LoopClosing · LoopDetector · LidarLoopDetector(stub) · GlobalJointBA
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
| `mapping/` | LocalMapping + LocalJointBA + **MapIncremental** + `ivox/`（live IVox 所有者） |
| `backend/` | LoopClosing + GlobalJointBA |
| `relocalization/` | Relocalizer（BoW **同步**于 Tracking） |
| `estimate/` | ResidualMask + residual_* |
| `data/` | 唯一稀疏图 State |
| `io/` | DenseMap / G2P5 等侧路 |
| `optimize/` | g2o，**冻结不膨胀** |
| `util/` | schedule / modality / publishers |

## Canonical 类名

| §2b | 主类名（旧名兼容别名） |
|-----|-----------|
| `frontend/tracking.*` | `Tracking`（`using tracking_module = Tracking`） |
| `mapping/local_mapping.*` | `LocalMapping`（`using mapping_module = LocalMapping`） |
| `backend/loop_closing.*` | `LoopClosing`（`using global_optimization_module = LoopClosing`） |
| `frontend/eskf/` | `frontend::Eskf` |
| `mapping/ivox/` | `mapping::IVox` |
| `sensor/lidar/.../preprocess` | `sensor::Preprocess` |
| `sensor/lidar/.../obs_model` | `sensor::ObsModel` |
| `mapping/local_joint_ba.hpp` | `mapping::LocalJointBA` |
| `backend/global_joint_ba.*` | `backend::GlobalJointBA` |
| `mapping/map_incremental.hpp` | `mapping::MapIncremental`（`IntegrateScan` 插入；LidarSensor `set_ivox` 只读） |
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

## Lidar P0 / P1 / P2 进度

| 项 | 状态 | 说明 |
|----|------|------|
| **P0.1** MapIncremental owns insert | ✅ | `IntegrateScan`；`FeedWithPose` 只建残差；`LidarBridge::SetMapIncremental` |
| **P0.2** Sync + deskew skeleton | ✅ | `frontend/lio/{measure_group,imu_process,sync}`；无点时则 pass-through |
| **P0.3** Stronger Eskf | ✅ | 15×15 `P_` + predict/`Joseph` 粗更新；完整 IEKF 仍缺（见 eskf 注释） |
| **P1** TimedPoint / IVox LRU / keyframe / `T_il` | ✅ | `RunTimed`；`max_voxels` 按插入序淘汰；`LidarKeyframeManager`；`SetT_imu_lidar` |
| **P2** Loop stub + ground PCA + docs | ✅ | `LidarLoopDetector::Detect→false`；`BuildStub` 最低 20% z PCA |

Deferred（相对 lightning-lm）：完整 IMU 离散 ΦQΦᵀ、IEKF 点面 H、NDT 回环、强度/时间戳驱动解码。

## 约定

- 无双系统 / PoseFusion / 独立 Lightning Start。
- 残差由 Pipeline `ResidualMask::FromRuntime` 注入 Local/Global Joint BA。
- JointBA：同 mask + State；单次 `optimize()` 可分阶段（vision/imu → lidar/odom refine）。
- ThreadPool：`Schedule` / `ScheduleAfter`（回环 BA 后 DenseMap `RequestRebuild`）。
- Reloc 保持 Frontend 同步（BoW 暂不 Schedule）。

## Runtime profiles

`conf/atlas/profiles/`：`vo` `vio` `lo` `lio` `livo` `wio` `lwio` `lvwio`。

新代码请 include §2b 路径与 `Tracking` / `LocalMapping` / `LoopClosing` 别名。
