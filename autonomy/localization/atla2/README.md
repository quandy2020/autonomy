# Atla2 — unified VO / VIO / LIO / LIVO SLAM

Config-driven SLAM for autonomy localization. One backend + map, four
frontends (`vo` | `vio` | `lio` | `livo`). Core code has **no ROS dependency**.

| | |
|---|---|
| Design | [docs/README.md](docs/README.md)（文档中心）· [docs/framework.md](docs/framework.md) |
| Proto | [proto/README.md](proto/README.md)（复用 automsgs） |
| License | [LICENSE](LICENSE) (Apache-2.0) |
| Build | parent `autonomy/localization` + this [CMakeLists.txt](CMakeLists.txt) |

## Layout

```text
atla2/
├── CMakeLists.txt       # apps binaries + config install
├── README.md
├── LICENSE
├── common/              # types, time, math, transform, config, thread
├── sensor/              # interface + camera/imu/lidar/gps/baro/flow/sync
├── frontend/            # vo / vio / lio / livo + factory
├── backend/             # IEKF + Ceres/graph + factors
├── map/                 # landmark / keyframe / elevation / occupancy / clouds
├── fusion/              # ekf / factor_graph / outlier / degradation
├── pipeline/            # SlamSystem + state machine
├── config/              # sensors / frontend / backend / map / platforms
├── apps/
│   ├── air_slam_node/
│   ├── offline_runner/
│   ├── calibration_tool/{camera_imu,lidar_imu,camera_lidar,joint}_calib/
│   └── benchmark/
├── thirdparty/          # policy stubs (prefer system packages)
├── test/{unit,integration,dataset,simulation}/
├── scripts/             # build.sh / calibrate.sh / evaluate.py
├── tools/               # trajectory_eval / map_visualizer / log_analyzer / benchmark
├── proto/               # VO/VIO/LO/LIO/LIVO 消息（复用 automsgs）
└── docs/                # 设计·需求·接口·部署·测试·标定（见 docs/README.md）
```

## Build

Sources under `atla2/` are compiled into `autonomy_localization` via
`autonomy_glob_srcs`. Apps are registered by `atla2/CMakeLists.txt`
(`add_subdirectory(atla2)` from `localization/CMakeLists.txt`).

```bash
# configure + build (workspace root)
ninja -C build autonomy.localization.atla2_offline

# unit tests (when BUILD_TEST=ON; *_test.cpp auto-collected)
ctest -R atla2 -C build --output-on-failure
```

Helper scripts:

```bash
./scripts/build.sh          # wraps ninja for atla2 targets
./scripts/calibrate.sh      # launches calibration_tool CLIs
python3 ./scripts/evaluate.py --help
```

## Run (synthetic offline)

```bash
./build/bin/autonomy.localization.atla2_offline \
  --config src/autonomy/autonomy/localization/atla2/config/platforms/drone_livo.yaml \
  --steps 200
```

### Platforms

| File | Mode |
|------|------|
| `config/platforms/drone_vo.yaml` | VO |
| `config/platforms/drone_vision_only.yaml` | VIO |
| `config/platforms/drone_lo.yaml` | LO（纯激光） |
| `config/platforms/drone_lidar_imu.yaml` | LIO |
| `config/platforms/drone_livo.yaml` | LIVO (loose) |
| `config/platforms/vtol_livo_gps.yaml` | LIVO + GPS |

Sensor noise / extrinsics live under `config/sensors/{camera,imu,lidar,gps,barometer,optical_flow}/`.

## API sketch

```cpp
#include "autonomy/localization/atla2/pipeline/slam_system.hpp"

autonomy::localization::atla2::SlamSystem slam;
slam.InitFromFile(".../drone_livo.yaml");
slam.PushImu(imu);
slam.PushImage(img);
slam.PushLidar(scan);
while (slam.Step()) {
  autonomy::localization::atla2::OdometryResult odom;
  slam.GetOdometry(&odom);
}
```

## Dependencies

Prefer workspace / system packages (see `thirdparty/README.md`):

- Eigen3, Ceres Solver, OpenCV (via localization `FEATURES slam`)
- Optional later: Sophus, g2o, GTSAM (not required for current skeleton)

## Status

| Module | Status |
|--------|--------|
| Interfaces + factory + config | done |
| Sensor tree (camera/imu/lidar/…) | done |
| VO / VIO / LIO / LIVO skeleton | done |
| IEKF + Ceres sliding-window VIO | done |
| Fusion (ekf / gate / degradation) | skeleton |
| Map / pipeline / offline app | done |
| Calibration tools / online node | stub |
| Feature tracker / ICP / tight LIVO | TODO |
| Loop closure | TODO |
| Autolink / ROS bridge | TODO |
