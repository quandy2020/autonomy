# autonomy::control::tools

ROS-free algorithm cores ported from:

- [ros-controls/control_toolbox](https://github.com/ros-controls/control_toolbox)
- [ros-controls/ros2_controllers](https://github.com/ros-controls/ros2_controllers)

**Coverage status (algorithm-core scope): complete.**  
All portable math / filters / odometry / trajectory / limiter / admittance / gripper-tracker cores from the upstream packages that do not require ROS middleware have been ported. Remaining upstream surface is intentionally ROS-bound (see 「无法移植」 below).

## control_toolbox

Namespace: `autonomy::control::tools`  
Upstream: `control_toolbox/control_toolbox/`  
Include style: `#include "autonomy/control/tools/<name>.hpp"`

| Module | Headers / sources | Upstream |
|--------|-------------------|----------|
| PID | `pid.hpp`, `pid.cpp`, `realtime_box.hpp` | `include/control_toolbox/pid.hpp` |
| Multi-DOF PID | `multi_dof_pid.hpp` | Stand-in for `pid_controller` DOF loop over `Pid` |
| Filters (fn) | `filters.hpp` (`exponentialSmoothing`) | `include/control_toolbox/filters.hpp` |
| Exponential filter | `exponential_filter.hpp` | `control_filters/exponential_filter` (no FilterBase) |
| Filter chain | `filter_chain.hpp` | Sequential EMA / callable stages (no FilterBase / pluginlib) |
| Gravity compensation | `gravity_compensation.hpp` | `control_filters/gravity_compensation` (Eigen only; no TF) |
| Rate limiter | `rate_limiter.hpp` | `include/control_toolbox/rate_limiter.hpp` |
| Rolling mean | `rolling_mean.hpp` | `rcpputils` / `rcppmath` rolling mean |
| Dither | `dither.hpp`, `dither.cpp` | `include/control_toolbox/dither.hpp` |
| Limited proxy | `limited_proxy.hpp`, `limited_proxy.cpp` | `include/control_toolbox/limited_proxy.hpp` |
| Sine sweep | `sine_sweep.hpp`, `sine_sweep.cpp` | `include/control_toolbox/sine_sweep.hpp` |
| Sinusoid | `sinusoid.hpp`, `sinusoid.cpp` | `include/control_toolbox/sinusoid.hpp` |
| Low-pass filter | `low_pass_filter.hpp` | `include/control_toolbox/low_pass_filter.hpp` |

## ros2_controllers (math / odometry / trajectory / limiters)

Namespace: `autonomy::control::tools::controllers::<name>`  
Path: `controllers/`  
Include style: `#include "autonomy/control/tools/controllers/<pkg>/<file>.hpp"`

| Package | Ported | Notes |
|---------|--------|-------|
| `diff_drive` | `odometry.hpp/cpp`, `speed_limiter.hpp`, `kinematics.hpp` | `rclcpp::Time` → `double time_sec`; speed limiter wraps `tools::RateLimiter`; IK/FK: `left/right = (v ± ω L/2) / r` |
| `mecanum_drive` | `odometry.hpp/cpp` | `tf2` planar math inlined |
| `omni_wheel_drive` | `odometry.hpp/cpp` | Eigen SVD FK kept |
| `joint_trajectory` | `trajectory.hpp/cpp`, `interpolation_methods.hpp`, `tolerances.hpp`, `trajectory_types.hpp`, `trajectory_tracker.hpp` | POD `TrajectoryPoint` / `JointTrajectory`; tracker SM: Idle/Tracking/Hold/Stopping/Succeeded/Aborted (no action server) |
| `steering` | `steering_kinematics.hpp/cpp`, `twist_to_steering.hpp` | From `steering_controllers_library`; bicycle / tricycle / ackermann FK+IK; thin `twist_to_steering` → `get_commands` |
| `tricycle` | `odometry.hpp/cpp`, `steering_limiter.hpp/cpp`, `traction_limiter.hpp/cpp` | From `tricycle_controller`; `update(Ws, alpha, dt_sec)` matches upstream `.cpp` (wheel ω + steer angle) |
| `admittance` | `admittance_rule.hpp` | 6DOF Cartesian `X_ddot = M^{-1}(F - D X_dot - K X)`; optional joint helpers via `std::function`; no kinematics_interface / ParamListener |
| `gripper` | `gripper_command.hpp` | Goal `{position, max_effort}` + Idle/Moving/Succeeded/Stalled (upstream `check_for_success` math; no action server) |

### Steering layout note (bicycle / ackermann / tricycle_steering)

There are **no** separate `controllers/bicycle/`, `controllers/ackermann/`, or `controllers/tricycle_steering/` trees.  
Use `controllers/steering/SteeringKinematics` (or `twist_to_steering`) with:

- `BICYCLE_CONFIG` — single traction + single steer (bicycle / `bicycle_steering_controller` math)
- `TRICYCLE_CONFIG` — dual traction + single steer (`tricycle_steering_controller` math)
- `ACKERMANN_CONFIG` — dual traction + dual steer (`ackermann_steering_controller` math)

Dedicated `tricycle_controller` (single driven steered wheel) odometry / limiters live under `controllers/tricycle/`.

## Adaptation notes

- `rclcpp::Duration` / `rclcpp::Time` → `double` seconds (prefer `time_sec` / `dt` / `dt_sec` APIs)
- `realtime_tools::RealtimeThreadSafeBox` → local `RealtimeThreadSafeBox` in `realtime_box.hpp`
- `geometry_msgs::WrenchStamped` low-pass specialization → `Eigen::Matrix<double, 6, 1>`
- Gravity compensation: caller passes `R_sensor_from_world` (no `tf2_ros`)
- Admittance: caller supplies wrench in base + `rot_base_control`; optional `CartToJoint` / `JointToCart` callbacks replace `kinematics_interface`
- `RollingMeanAccumulator` → `autonomy/control/tools/rolling_mean.hpp`
- Include path style: `#include "autonomy/control/tools/<name>.hpp"`

---

## 无法移植（ROS-bound / out of algorithm-core scope）

These upstream pieces are **not** ported and are not planned under `control/tools` without a ROS-free redesign:

### control_toolbox

| Item | Reason |
|------|--------|
| `pid_ros.*` | ROS parameter / node bindings |
| `control_filters` plugin wrappers | `filters::FilterBase`, pluginlib, `generate_parameter_library` |
| TF-driven filter plumbing | `tf2_ros::Buffer` / TransformListener (math core is in `gravity_compensation.hpp`) |
| Tests, srv, docs, `package.xml`, ament CMake | Build / test infrastructure |

### ros2_controllers

| Item | Reason |
|------|--------|
| All `*Controller` / `*Broadcaster` classes | `controller_interface`, hardware_interface, lifecycle, params |
| `admittance_controller` full class | Needs `kinematics_interface` + ROS params / msgs; **algorithm core** is in `controllers/admittance/admittance_rule.hpp` |
| `pid_controller` full class | ChainableControllerInterface + msgs; use `multi_dof_pid.hpp` for the DOF loop |
| `forward_command_controller`, `effort/position/velocity_controllers` | Thin ROS command passthrough |
| `gripper_*` / `parallel_gripper_*` action + HW adapters | Action server / hardware_interface; **tracker core** is in `controllers/gripper/gripper_command.hpp` |
| `force_torque_*`, `imu_*`, `gpio_*`, `range_*`, `gps_*`, `pose_*`, `battery_*`, `magnetometer_*`, `state_interfaces_*`, `joint_state_broadcaster` | Sensor / broadcaster / HW plumbing |
| `chained_filter_controller`, `motion_primitives_controllers` | Controller framework (`filter_chain.hpp` covers sequential scalar filtering only) |
| `joint_trajectory` ROS helpers | `get_segment_tolerances` from params / `FollowJointTrajectory` action; **tracker SM** is in `trajectory_tracker.hpp` |
| Deprecated `steering_odometry` wrapper | Use `steering::SteeringKinematics` / `twist_to_steering` |
| Tests, docs, `package.xml`, ament CMake | Build / test infrastructure |

---

## File map (ported)

```
tools/
  pid.hpp / pid.cpp
  multi_dof_pid.hpp
  realtime_box.hpp
  filters.hpp
  exponential_filter.hpp
  filter_chain.hpp
  gravity_compensation.hpp
  rate_limiter.hpp
  rolling_mean.hpp
  dither.hpp / dither.cpp
  limited_proxy.hpp / limited_proxy.cpp
  sine_sweep.hpp / sine_sweep.cpp
  sinusoid.hpp / sinusoid.cpp
  low_pass_filter.hpp
  controllers/
    admittance/       admittance_rule.hpp
    diff_drive/       odometry.*, speed_limiter.hpp, kinematics.hpp
    mecanum_drive/    odometry.*
    omni_wheel_drive/ odometry.*
    joint_trajectory/ trajectory.*, interpolation_methods.hpp, tolerances.hpp,
                      trajectory_types.hpp, trajectory_tracker.hpp
    steering/         steering_kinematics.*, twist_to_steering.hpp
    tricycle/         odometry.*, steering_limiter.*, traction_limiter.*
    gripper/          gripper_command.hpp
```
