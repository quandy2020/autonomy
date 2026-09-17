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
namespace planner {

/**
 * @brief Effective translational velocity (m/s) after velocity_scale.
 *
 * Prefer @c request.pb.cartesian_limits() when configured; else @c max_velocity.
 */
inline double EffectiveTransVel(const MotionPlanRequest& request) {
  const double scale = std::max(1e-3, request.pb.velocity_scale());
  if (request.pb.cartesian_limits().configured() &&
      request.pb.cartesian_limits().max_translational_velocity() > 0.0) {
    return request.pb.cartesian_limits().max_translational_velocity() * scale;
  }
  return std::max(1e-6, request.pb.max_velocity() * scale);
}

/** @brief Effective translational acceleration (m/s²). */
inline double EffectiveTransAcc(const MotionPlanRequest& request) {
  const double scale = std::max(1e-3, request.pb.acceleration_scale());
  if (request.pb.cartesian_limits().configured() &&
      request.pb.cartesian_limits().max_translational_acceleration() > 0.0) {
    return request.pb.cartesian_limits().max_translational_acceleration() * scale;
  }
  return std::max(1e-6, request.pb.max_acceleration() * scale);
}

/** @brief Effective translational deceleration (m/s²). */
inline double EffectiveTransDec(const MotionPlanRequest& request) {
  const double scale = std::max(1e-3, request.pb.acceleration_scale());
  if (request.pb.cartesian_limits().configured() &&
      request.pb.cartesian_limits().max_translational_deceleration() > 0.0) {
    return request.pb.cartesian_limits().max_translational_deceleration() * scale;
  }
  return EffectiveTransAcc(request);
}

/** @brief Effective rotational velocity (rad/s) after velocity_scale. */
inline double EffectiveRotVel(const MotionPlanRequest& request) {
  const double scale = std::max(1e-3, request.pb.velocity_scale());
  if (request.pb.cartesian_limits().configured() &&
      request.pb.cartesian_limits().max_rotational_velocity() > 0.0) {
    return request.pb.cartesian_limits().max_rotational_velocity() * scale;
  }
  return std::max(1e-6, request.pb.max_velocity() * scale);
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

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
