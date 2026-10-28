# Autonomy Atlas visual SLAM

This directory is the autonomy-localization-owned ORB-SLAM3 integration. It is
self-contained and does not depend on the repository's top-level Voxel Atlas.

The estimator is organized by responsibility:

- `atlas/sensor/`: frames, keyframes, map points, IMU data and camera models.
- `atlas/feature/`: ORB extraction and the FBoW vocabulary adapter.
- `atlas/match/`: ORB correspondence search plus PnP and Sim3 match validation.
- `atlas/tracking/`, `atlas/mapping/`, `atlas/closure/`: the SLAM front end,
  local mapping and loop-closing pipelines.
- `atlas/map/` and `atlas/optimization/`: map ownership and g2o optimization.
- `atlas/common/io/`: configuration and serialization support.
- `atlas/common/math/`: shared geometry and conversion utilities.
- `atlas/`: the ORB-SLAM3 runtime, drawers and headless viewer.
- `atlas/visual_slam_message_adapter.*` and `atlas/atlas_node.*`: AutoMsgs and
  Autolink integration.
- `thirdparty/`: FBow headers plus the private g2o and Sophus sources.
- `config/`: reference camera configurations and the binary FBoW vocabulary.

Link `autonomy::localization_atlas` and use `atlas/map_builder.hpp` for the
public C++ API. It supports monocular, stereo, RGB-D, monocular-inertial,
stereo-inertial and RGB-D-inertial tracking.

ORB-SLAM3 is GPLv3 software. Its license is preserved in `COPYING`; bundled
third-party notices remain beside their respective sources and in
`THIRD_PARTY.md`.
