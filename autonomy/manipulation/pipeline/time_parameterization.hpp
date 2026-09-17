/*
 * Copyright 2026 The Openbot Authors
 *
 * Time-optimal trajectory generation (Kunz–Stilman style).
 */

#pragma once

#include <automsgs/msgs/trajectory_msgs/joint_trajectory.pb.h>

#include "autonomy/manipulation/proto/time_parameterization_options.pb.h"

namespace autonomy {
namespace manipulation {
namespace trajectory {

/** @brief Proto options for time parameterization (alias for callers). */
using TimeParameterizationOptions =
    ::autonomy::manipulation::proto::TimeParameterizationOptions;

/**
 * @brief Kunz–Stilman time-optimal trajectory generation: rewrite time_from_start
 *        under maximum velocity / acceleration.
 */
bool ApplyTimeOptimalTrajectoryGeneration(
    automsgs::msgs::trajectory_msgs::JointTrajectory* trajectory,
    const TimeParameterizationOptions& options);

/**
 * @brief Optional Ruckig smooth; falls back to time-optimal generation if the
 *        library is missing or smoothing fails.
 */
bool ApplyRuckigTrajectorySmoothing(
    automsgs::msgs::trajectory_msgs::JointTrajectory* trajectory,
    const TimeParameterizationOptions& options);

/** @brief Ruckig smooth with default TimeParameterizationOptions. */
bool ApplyRuckigTrajectorySmoothing(
    automsgs::msgs::trajectory_msgs::JointTrajectory* trajectory);

}  // namespace trajectory
}  // namespace manipulation
}  // namespace autonomy
