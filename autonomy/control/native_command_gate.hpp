/* Copyright 2026 The Openbot Authors. */
/** @file native_command_gate.hpp
 * @brief Independent measured-map gate for the native ControllerServer command transport.
 */
#pragma once

#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include "autonomy/perception/data/map_reader.hpp"
#include "opt/navigation/safety.h"

namespace autonomy::control {
/** @brief Explicit platform envelope, feedback and actual actuator reaction bounds. */
struct NativeCommandGateOptions {
    opt::navigation::SafetyOptions body;  ///< Existing full-footprint/height/ground-step policy.
    double maximum_linear_speed = .4;  ///< Actual command envelope, m/s.
    double maximum_yaw_rate = .8;  ///< Actual yaw command envelope, rad/s.
    double maximum_feedback_age = .25;  ///< Original calibrated feedback age, seconds.
    double maximum_map_age = 1;  ///< Original actual mapping acquisition age, seconds.
    double actuator_watchdog = .5;  ///< Actual base maximum stale-command interval, seconds.
    double reaction_time = .1;  ///< Observation/dispatch/brake reaction allowance, seconds.
    double minimum_braking_deceleration = .5;  ///< Actual conservative physical braking lower bound, m/s^2.
};

/** @brief Check one native proposal against original feedback and actual conservative map geometry.
 * A circular envelope covers every yaw and encloses all travel until watchdog,
 * reaction and braking complete. The existing GeometrySafety checks all cells
 * touched by that enlarged footprint, including height/step and unknown space.
 * This voxel/plane sensing-model admission is not an actuator-stop receipt or
 * a proof that unseen/dynamic/subcell obstacles do not exist. Caller serializes
 * source selection and publishes only a command for which this returns safe.
 */
opt::navigation::SafetyResult VerifyNativeCommand(
    const automsgs::msgs::geometry_msgs::TwistStamped& command,
    const automsgs::msgs::nav_msgs::Odometry& actual_feedback,
    const perception::MapReadView& map, const NativeCommandGateOptions& options,
    double now);
}  // namespace autonomy::control
