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

/** @brief Physical bounds for the explicitly configured startup yaw-only admission. */
struct NativeStartupRotationOptions {
    double maximum_position_drift = .03;  ///< Maximum feedback translation from the startup pose, metres.
    double maximum_yaw_rate = .8;  ///< Absolute yaw command bound, radians per second.
    double maximum_feedback_age = .25;  ///< Original calibrated feedback age, seconds.
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
/** @brief Admit only zero-linear yaw commands inside one deployment-confirmed startup envelope.
 * This check intentionally does not consult unknown map space because the robot
 * already occupies that body envelope. It never admits translation, never
 * creates ground evidence, and expires with the supplied absolute deadline.
 * @param command Proposed native planar twist.
 * @param feedback Current healthy calibrated odometry from the same acquisition.
 * @param start_x Startup envelope X in the feedback frame.
 * @param start_y Startup envelope Y in the feedback frame.
 * @param deadline Absolute owner-clock deadline for the startup sweep.
 * @param options Drift, yaw-rate and freshness bounds.
 * @param now Current owner clock in the feedback stamp domain.
 * @return Safe only for a fresh, pure yaw command with zero linear translation.
 */
opt::navigation::SafetyResult VerifyNativeStartupRotation(
    const automsgs::msgs::geometry_msgs::TwistStamped& command,
    const automsgs::msgs::nav_msgs::Odometry& feedback, double start_x,
    double start_y, double deadline,
    const NativeStartupRotationOptions& options, double now);
}  // namespace autonomy::control
