# atlas/sensor

Single ingress for measurements. **Not** a second SLAM.

**All ROS / Autolink IO enters via `sensor/*/ *Bridge`:**

| Bridge | Path | Feeds |
|--------|------|-------|
| `CameraBridge` | `camera/camera_bridge.*` | `system::feed_*_frame` + `CameraSensor` |
| `ImuBridge` | `imu/imu_bridge.*` | `ImuSensor` / `LocalEstimator` / `system::feed_imu` |
| `LidarBridge` | `lidar/lidar_bridge.*` | `LidarSensor` (+ pose pub) |
| `OdomBridge` | `odom/odom_bridge.*` | `OdomSensor` |

| Dir | Role |
|-----|------|
| `imu/` | Buffer + preintegrator + ImuBridge |
| `camera/` | Models + CameraSensor + CameraBridge (vision ROS IO) |
| `lidar/` | Cloud queue + LidarBridge; `lightning/` = embedded LIO algo |
| `odom/` | Wheel / external odometry + OdomBridge |
| `sensor_suite.*` | Starts enabled sensors from `RuntimeConfig` |

Upper layers (frontend / mapping / backend) only consume these sources.
Root `atlas/image_bridge.hpp` is a thin compat shim → `camera/camera_bridge.hpp`.
