/*
 * Copyright 2026 The Openbot Authors
 *
 * Time-optimal trajectory generation (Kunz–Stilman style).
 *
 * Path parameterized in joint space; forward/backward velocity limits under
 * per-joint vmax and amax. Falls back to constant-vmax timing on degeneracy.
 */

#pragma once

#include <vector>

#include "autonomy/manipulation/model/robot_model.hpp"

namespace autonomy {
namespace manipulation {
namespace trajectory {

/** @brief Velocity / acceleration limits for time parameterization. */
struct TimeParamOptions {
  double max_velocity = 1.0;
  double max_acceleration = 2.0;
  /** @brief Optional per-joint vmax (empty → use max_velocity). */
  std::vector<double> max_velocity_vector;
  /** @brief Optional per-joint amax (empty → use max_acceleration). */
  std::vector<double> max_acceleration_vector;
  /**
   * @brief Max deviation for CircularPathSegment blends at waypoints (Kunz).
   * Units match joint-space path; ≤0 falls back to 0.1.
   */
  double path_tolerance = 0.1;
  /** @brief Continuous TOTG integration step (path parameter units / step). */
  double integration_time_step = 0.001;
};

/**
 * @brief Kunz–Stilman TOTG: rewrite time_from_start under vmax/amax.
 * @param[in,out] traj Trajectory whose timestamps are updated in place.
 * @param[in] options Max velocity / acceleration (scalar or per-joint).
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
