# autonomy::control::tools

ROS-free algorithm cores ported from:

- [ros-controls/control_toolbox](https://github.com/ros-controls/control_toolbox)
- [ros-controls/ros2_controllers](https://github.com/ros-controls/ros2_controllers)

## control_toolbox

Namespace: `autonomy::control::tools`  
Upstream: `control_toolbox/control_toolbox/`

| Module | Headers / sources | Upstream |
|--------|-------------------|----------|
| PID | `pid.hpp`, `pid.cpp`, `realtime_box.hpp` | `include/control_toolbox/pid.hpp` |
| Filters | `filters.hpp` (`exponentialSmoothing`) | `include/control_toolbox/filters.hpp` |
| Rate limiter | `rate_limiter.hpp` | `include/control_toolbox/rate_limiter.hpp` |
| Rolling mean | `rolling_mean.hpp` | `rcpputils` / `rcppmath` rolling mean |
| Dither | `dither.hpp`, `dither.cpp` | `include/control_toolbox/dither.hpp` |
| Limited proxy | `limited_proxy.hpp`, `limited_proxy.cpp` | `include/control_toolbox/limited_proxy.hpp` |
| Sine sweep | `sine_sweep.hpp`, `sine_sweep.cpp` | `include/control_toolbox/sine_sweep.hpp` |
| Sinusoid | `sinusoid.hpp`, `sinusoid.cpp` | `include/control_toolbox/sinusoid.hpp` |
| Low-pass filter | `low_pass_filter.hpp` | `include/control_toolbox/low_pass_filter.hpp` |

### Intentionally skipped (control_toolbox)

- `pid_ros.*` (ROS parameter / node bindings)
- `control_filters/*` (pluginlib / generate_parameter_library / filters package)
- Tests, srv, docs, `package.xml`, ament CMake

## ros2_controllers (math / odometry / trajectory only)

Namespace: `autonomy::control::tools::controllers::<name>`  
Path: `controllers/`  
Include style: `#include "autonomy/control/tools/controllers/diff_drive/odometry.hpp"`

| Package | Ported | Notes |
|---------|--------|-------|
| `diff_drive` | `odometry.hpp/cpp`, `speed_limiter.hpp` | `rclcpp::Time` → `double time_sec`; speed limiter wraps `tools::RateLimiter` |
| `mecanum_drive` | `odometry.hpp/cpp` | `tf2` planar math inlined |
| `omni_wheel_drive` | `odometry.hpp/cpp` | Eigen SVD FK kept |
| `joint_trajectory` | `trajectory.hpp/cpp`, `interpolation_methods.hpp`, `tolerances.hpp`, `trajectory_types.hpp` | POD `TrajectoryPoint` / `JointTrajectory` (no `trajectory_msgs`); times are `double` seconds |
| `steering` | `steering_kinematics.hpp/cpp` | From `steering_controllers_library`; bicycle / tricycle / ackermann FK+IK |

### Intentionally skipped (ros2_controllers)

- Full `*Controller` classes (`ChainableControllerInterface`, hardware_interface, lifecycle, parameter libraries)
- `pid_controller`, `forward_command_controller`, `gripper_*`, `force_torque_*`, etc.
- `joint_trajectory` ROS helpers: `get_segment_tolerances` (params / `FollowJointTrajectory` action)
- Deprecated `steering_odometry` wrapper (use `steering::SteeringKinematics`)
- `tricycle_controller` odometry (not requested; kinematics covered via steering)
- Tests, docs, `package.xml`, ament CMake

## Adaptation notes

- `rclcpp::Duration` / `rclcpp::Time` → `double` seconds (prefer `time_sec` / `dt` APIs)
- `realtime_tools::RealtimeThreadSafeBox` → local `RealtimeThreadSafeBox` in `realtime_box.hpp`
- `geometry_msgs::WrenchStamped` low-pass specialization → `Eigen::Matrix<double, 6, 1>`
- `RollingMeanAccumulator` → `autonomy/control/tools/rolling_mean.hpp`
- Include path style: `#include "autonomy/control/tools/<name>.hpp"`
