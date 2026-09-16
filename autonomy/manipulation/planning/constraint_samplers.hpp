/*
 * Copyright 2026 The Openbot Authors
 *
 * Constraint checkers for MotionPlanRequest (MoveIt kinematic_constraints).
 */

#pragma once

#include <cmath>
#include <string>

#include "autonomy/manipulation/core/error_codes.hpp"
#include "autonomy/manipulation/planning/planner_base.hpp"

namespace autonomy {
namespace manipulation {
namespace constraint_samplers {

/**
 * @brief Check whether @p state satisfies all joint constraints in @p req.
 * @param[in] req Request carrying joint constraints.
 * @param[in] state Joint state to test.
 * @return true if every constraint is met (or no constraints).
 */
inline bool SatisfiesJointConstraints(const planning::MotionPlanRequest& req,
                                      const core::JointState& state) {
  for (const auto& c : req.joint_constraints) {
    bool found = false;
    for (std::size_t i = 0; i < state.names.size(); ++i) {
      if (state.names[i] != c.joint_name || i >= state.positions.size()) {
        continue;
      }
      found = true;
      const double q = state.positions[i];
      if (q < c.position - c.tolerance_below ||
          q > c.position + c.tolerance_above) {
        return false;
      }
    }
    if (!found) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Check a tip pose against a position constraint.
 * @param[in] c Position constraint.
 * @param[in] pose Tip pose to test.
 * @return true if Euclidean distance to target ≤ tolerance.
 */
inline bool SatisfiesPositionConstraint(const planning::PositionConstraint& c,
                                        const kinematics::Pose& pose) {
  const double dx = pose.x - c.target.x;
  const double dy = pose.y - c.target.y;
  const double dz = pose.z - c.target.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz) <= c.tolerance;
}

/**
 * @brief Check a tip pose against an orientation constraint.
 * @param[in] c Orientation constraint (tolerance in radians).
 * @param[in] pose Tip pose to test.
 * @return true if quaternion angle to target ≤ tolerance.
 */
inline bool SatisfiesOrientationConstraint(
    const planning::OrientationConstraint& c, const kinematics::Pose& pose) {
  // Quaternion dot |q·t| ≈ 1 → angle ≈ 0; threshold via tolerance (rad).
  const double dot = std::abs(pose.qw * c.target.qw + pose.qx * c.target.qx +
                              pose.qy * c.target.qy + pose.qz * c.target.qz);
  const double clamped = std::min(1.0, std::max(0.0, dot));
  const double angle = 2.0 * std::acos(clamped);
  return angle <= c.tolerance;
}

/**
 * @brief Check that every waypoint satisfies joint and Cartesian path constraints.
 * @param[in] req Request carrying joint / position / orientation constraints.
 * @param[in] traj Trajectory to validate.
 * @return true if all waypoints are valid.
 */
inline bool SatisfiesPathConstraints(const planning::MotionPlanRequest& req,
                                     const core::RobotTrajectory& traj) {
  for (const auto& wp : traj.waypoints) {
    if (!SatisfiesJointConstraints(req, wp)) {
      return false;
    }
    if (req.kinematics &&
        (!req.position_constraints.empty() ||
         !req.orientation_constraints.empty())) {
      kinematics::Pose tip;
      if (!req.kinematics->GetPositionFK(wp, &tip)) {
        continue;
      }
      for (const auto& c : req.position_constraints) {
        if (!SatisfiesPositionConstraint(c, tip)) {
          return false;
        }
      }
      for (const auto& c : req.orientation_constraints) {
        if (!SatisfiesOrientationConstraint(c, tip)) {
          return false;
        }
      }
    }
  }
  return true;
}

/**
 * @brief Pre-plan check: start / goal joint constraints (+ optional FK pose).
 * @param[in] req Planning request.
 * @param[out] error Optional human-readable failure reason.
 * @return ErrorCode::kSuccess or a constraint-violation code.
 */
inline ErrorCode EvaluateRequestConstraints(
    const planning::MotionPlanRequest& req, std::string* error) {
  if (!SatisfiesJointConstraints(req, req.start_state)) {
    if (error) {
      *error = "start state violates joint constraints";
    }
    return ErrorCode::kStartStateViolatesPathConstraints;
  }
  if (!req.goal_state.positions.empty() &&
      !SatisfiesJointConstraints(req, req.goal_state)) {
    if (error) {
      *error = "goal state violates joint constraints";
    }
    return ErrorCode::kGoalViolatesPathConstraints;
  }
  if (req.kinematics &&
      (!req.position_constraints.empty() ||
       !req.orientation_constraints.empty())) {
    kinematics::Pose tip;
    if (req.kinematics->GetPositionFK(req.goal_state.positions.empty()
                                          ? req.start_state
                                          : req.goal_state,
                                      &tip)) {
      for (const auto& c : req.position_constraints) {
        if (!SatisfiesPositionConstraint(c, tip)) {
          if (error) {
            *error = "pose violates position constraint on " + c.link_name;
          }
          return ErrorCode::kGoalViolatesPathConstraints;
        }
      }
      for (const auto& c : req.orientation_constraints) {
        if (!SatisfiesOrientationConstraint(c, tip)) {
          if (error) {
            *error = "pose violates orientation constraint on " + c.link_name;
          }
          return ErrorCode::kGoalViolatesPathConstraints;
        }
      }
    }
  }
  return ErrorCode::kSuccess;
}

}  // namespace constraint_samplers
}  // namespace manipulation
}  // namespace autonomy
