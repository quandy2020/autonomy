# LIVO Modular Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor `autonomy/localization/livo` into modular, low-coupling packages with professional full-word naming, one class per file, reusing `autonomy/common` + `autolink` + `automsgs` at I/O boundaries.

**Architecture:** Adopt **1A** (proto at I/O/config boundaries; Eigen hot-path internals) and **2A** (keep vendored Sophus/vikit behind adapters; expose `common::transform::Rigid3d` at public edges). Orchestrator owns pipelines; SensorBridge owns Autolink I/O only.

**Tech Stack:** C++17, Eigen, PCL, OpenCV, yaml-cpp via `ParamHandler`, autolink Readers/Writers, automsgs protobuf.

**Spec:** Chat design (1A+2A) — modular packages under `livo/`.

## Global Constraints

- One public class (or one free-function translation unit) per `.hpp`/`.cpp` pair
- No abbreviations in **new** type/file/function names (product namespace `livo` retained)
- Prefer `automsgs` messages for I/O; add `localization_msgs` proto only when no existing message fits
- Reuse `ParamHandler`, `BlockingQueue`, `logging`, `macros`, `Rigid3d`, atlas-style Image conversion patterns
- `third_party/` stays transitional; do not rename vendored vikit/sophus APIs

## Naming Map

| Old | New |
|-----|-----|
| `LIVMapper` | `MapBuilder` |
| `LivoBridge` | `SensorBridge` |
| `LivoMapperOptions` / `LivoNodeFlags` | `MapBuilderOptions` / `LivoRunnerFlags` |
| `Preprocess` | `PointCloudPreprocessor` |
| `ImuProcess` | `ImuProcessor` |
| `VIOManager` | `VisualOdometry` |
| `LidarVoxelMap` | `LidarVoxelMap` |
| `VoxelOctoTree` | `VoxelOctreeNode` |
| `VOXEL_LOCATION` | `VoxelGridCoordinate` |
| `VOXEL_POINTS` | `VisualVoxelBucket` |
| `pointWithVar` | `PointWithVar` |
| `StatesGroup` | `States` |
| `MeasureGroup` | `MeasureGroup` |
| `LidarMeasureGroup` | `LidarMeasureGroup` |
| `ImuData` | `automsgs::msgs::sensor_msgs::Imu` (+ stamp helpers) |
| `SLAM_MODE` | `SlamMode` |
| `EKF_STATE` | `EkfStage` |
| `LID_TYPE` | `LidarType` |

## Target Layout (cartographer-aligned)

```
livo/
  common/                # types/, math/, utils/, common_lib, ParamHandler loaders
  sensor/                # PointCloudPreprocessor, ImuProcessor
  mapping/               # MapBuilder
    lidar/               # voxel map estimator
    visual/              # VisualFrame, VisualFeature, VisualPoint, VisualOdometry, …
  bridge/                # SensorBridge, OdometryPublisher, NodeRunner, msg converters
  io/                    # reserved (bag / serialization; cartographer parity)
  proto/                 # reserved (livo-specific options proto if needed)
  third_party/           # Sophus + vikit_common (unchanged)
```

Transform / extrinsics: reuse `autonomy/common/transform` (no local `livo/transform`).

---

### Task 1: Types package + stamp helpers

**Files:**
- Create: `types/slam_operating_mode.hpp`, `estimation_pipeline_stage.hpp`, `lidar_sensor_model.hpp`
- Create: `types/estimator_state.hpp`, `point_with_covariance.hpp`
- Create: `types/visual_inertial_measurement_group.hpp`, `synchronized_sensor_package.hpp`
- Create: `types/imu_stamp.hpp` (GetImuStampSeconds / helpers)
- Create: `math/point_cloud_types.hpp` (from utils/types.h)
- Modify: `common_lib.hpp` → thin umbrella including new headers + deprecated aliases

- [ ] **Step 1:** Extract types into one-type-per-file headers
- [ ] **Step 2:** Replace `ImuData` usages with `sensor_msgs::Imu` shared_ptr + stamp helpers
- [ ] **Step 3:** Keep temporary `using` aliases so algorithm TUs still compile

### Task 2: Split lidar_mapping headers

**Files:**
- Create: `lidar_mapping/voxel_map_configuration.hpp`, `point_to_plane.hpp`, `voxel_plane.hpp`, `voxel_grid_coordinate.hpp`, `downsampled_point.hpp`, `voxel_octree_node.hpp`, `voxel_map_estimator.hpp`
- Modify: move implementations from `voxel_map.cpp` into matching `.cpp` files (estimator + octree at minimum)
- Delete or umbrella old `voxel_map.hpp`

### Task 3: Split visual_odometry

**Files:**
- Move/rename: `frame.*`, `feature.hpp`, `visual_point.*`
- Create: `sub_sparse_map.hpp`, `affine_warp.hpp`, `visual_point_voxel_bucket.hpp`, `visual_odometry_manager.hpp/.cpp`
- Rename `VIOManager` → `VisualOdometry` in cpp

### Task 4: Preprocessing + inertial rename

**Files:**
- `preprocessing/lidar_point_cloud_preprocessor.hpp/.cpp`
- `inertial/inertial_measurement_preprocessor.hpp/.cpp`

### Task 5: Configuration via ParamHandler

**Files:**
- `configuration/mapping_orchestrator_options.hpp/.cpp`
- `configuration/voxel_map_configuration_loader.hpp/.cpp`
- Load YAML through `autonomy::common::ParamHandler`

### Task 6: Node I/O split

**Files:**
- `conversion/point_cloud2_converter.hpp/.cpp`, `image_converter.hpp/.cpp`
- `node/odometry_publisher.hpp/.cpp`
- `node/sensor_bridge.hpp/.cpp`
- `node/node_runner.hpp/.cpp`
- `mapping/mapping_orchestrator.hpp/.cpp`

### Task 7: Wire LocalizationServer + CMake includes

**Files:**
- `localization_server.cpp/.hpp`, `localization_main.cpp`, `CMakeLists.txt`, `launch/livo.launch`
- Update include paths for new layout; keep `ROOT_DIR` / OpenMP defs

### Task 8: Remove umbrellas and old aliases

- [ ] Delete deprecated aliases once all call sites use new names
- [ ] Remove empty old files at package root

## Test plan

- Compile `autonomy_localization` / `autonomy.localization` when deps available
- Smoke: `--localization_mode=livo` starts and creates readers
- No behavior change intended in this refactor (structure/naming only)
