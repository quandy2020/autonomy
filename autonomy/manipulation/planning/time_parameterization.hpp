/*
 * Copyright 2026 The Openbot Authors
 *
 * Simplified time-optimal trajectory generation (constant vmax/amax).
 */

#pragma once

#include "autonomy/manipulation/core/robot_model.hpp"

namespace autonomy {
namespace manipulation {
namespace trajectory {

/** @brief Velocity / acceleration limits for time parameterization. */
struct TimeParamOptions {
  double max_velocity = 1.0;
  double max_acceleration = 2.0;
};

/**
 * @brief Rewrite time_from_start to respect velocity bounds (uniform per joint).
 * @param[in,out] traj Trajectory whose timestamps are updated in place.
 * @param[in] options Max velocity / acceleration.
 * @return true on success.
 */
bool ApplyTotg(core::RobotTrajectory* traj, const TimeParamOptions& options);

/**
 * @brief Optional Ruckig smooth; falls back to TOTG if library missing / fails.
 * @param[in,out] traj Trajectory to smooth.
 * @param[in] options Max velocity / acceleration.
 * @return true on success (including TOTG fallback).
 */
bool ApplyRuckig(core::RobotTrajectory* traj, const TimeParamOptions& options);

/**
 * @brief Ruckig smooth with default TimeParamOptions.
 * @param[in,out] traj Trajectory to smooth.
 * @return true on success.
 */
bool ApplyRuckig(core::RobotTrajectory* traj);

}  // namespace trajectory
}  // namespace manipulation
}  // namespace autonomy
