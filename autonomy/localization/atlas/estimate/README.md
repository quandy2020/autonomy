# atlas/estimate

Canonical residual API for Local / Global Joint BA (single AtlasSystem).

| Header | Role |
|--------|------|
| `residual_mask.hpp` | Runtime vision/imu/lidar/odom switches (`FromRuntime`) |
| `residual_vision.hpp` | Re-exports optimize SE3 reproj edges (`MonoReprojEdge`, …) |
| `residual_imu.hpp` | Re-exports `optimize::imu_g2o::preintegration_edge` |
| `residual_lidar.hpp` / `.cpp` | Point-plane refine helpers + factor batch |
| `residual_odom.hpp` / `.cpp` | Wheel / external odom delta refine |
| `lidar_residual_source.hpp` | `ILidarResidualSource` pull interface |
| `buffered_lidar_residual_source.hpp` | Thread-safe lidar factor queue |
| `estimate.hpp` | Umbrella include |

**Semantics:** LocalJointBA and GlobalJointBA share the same `ResidualMask` and
map `State`. One `optimize()` may be multi-stage (vision/imu then lidar/odom);
that is the Atlas LIVO joint problem. Pure LO/LIO uses `frontend::LocalEstimator`
(ESKF) without Joint BA.
