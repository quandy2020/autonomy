/*
 * Copyright 2026 The Openbot Authors
 *
 * Pilz joint / Cartesian limits (MoveIt LimitsContainer lite).
 */

#pragma once

#include <algorithm>
#include <cmath>

#include "autonomy/manipulation/planner/pilz/pilz_blend.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {

/**
 * @brief Effective translational velocity (m/s) after velocity_scale.
 *
 * Prefer @c request.cartesian_limits when configured; else @c max_velocity.
 */
inline double EffectiveTransVel(const MotionPlanRequest& request) {
  const double scale = std::max(1e-3, request.velocity_scale);
  if (request.cartesian_limits.configured &&
      request.cartesian_limits.max_trans_vel > 0.0) {
    return request.cartesian_limits.max_trans_vel * scale;
  }
  return std::max(1e-6, request.max_velocity * scale);
}

/** @brief Effective translational acceleration (m/s²). */
inline double EffectiveTransAcc(const MotionPlanRequest& request) {
  const double scale = std::max(1e-3, request.acceleration_scale);
  if (request.cartesian_limits.configured &&
      request.cartesian_limits.max_trans_acc > 0.0) {
    return request.cartesian_limits.max_trans_acc * scale;
  }
  return std::max(1e-6, request.max_acceleration * scale);
}

/** @brief Effective translational deceleration (m/s²). */
inline double EffectiveTransDec(const MotionPlanRequest& request) {
  const double scale = std::max(1e-3, request.acceleration_scale);
  if (request.cartesian_limits.configured &&
      request.cartesian_limits.max_trans_dec > 0.0) {
    return request.cartesian_limits.max_trans_dec * scale;
  }
  return EffectiveTransAcc(request);
}

/** @brief Effective rotational velocity (rad/s) after velocity_scale. */
inline double EffectiveRotVel(const MotionPlanRequest& request) {
  const double scale = std::max(1e-3, request.velocity_scale);
  if (request.cartesian_limits.configured &&
      request.cartesian_limits.max_rot_vel > 0.0) {
    return request.cartesian_limits.max_rot_vel * scale;
  }
  return std::max(1e-6, request.max_velocity * scale);
}

/**
 * @brief Duration for a Cartesian stroke (translation + optional rotation).
 *
 * Takes max of translational ATRAP time and |Δθ|/ω_max.
 */
inline double CartesianStrokeDuration(const MotionPlanRequest& request,
                                      double trans_distance,
                                      double rot_angle) {
  const double tv = EffectiveTransVel(request);
  const double ta = EffectiveTransAcc(request);
  const double td = EffectiveTransDec(request);
  const double t_trans = AtrapDuration(trans_distance, tv, ta, td);
  const double rv = EffectiveRotVel(request);
  const double t_rot = std::abs(rot_angle) / std::max(1e-6, rv);
  return std::max({0.1, t_trans, t_rot});
}

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
