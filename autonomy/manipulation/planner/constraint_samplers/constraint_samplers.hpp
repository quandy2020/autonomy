/*
 * Copyright 2026 The Openbot Authors
 *
 * Constraint checkers for MotionPlanRequest (MoveIt kinematic_constraints).
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "autonomy/manipulation/model/error_codes.hpp"
#include "autonomy/manipulation/common/joint_state_util.hpp"
#include "autonomy/manipulation/common/planner_interface.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"

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
    for (int i = 0; i < state.name_size(); ++i) {
      if (state.name(i) != c.joint_name() || i >= state.position_size()) {
        continue;
      }
      found = true;
      const double q = state.position(i);
      if (q < c.position() - c.tolerance_below() ||
          q > c.position() + c.tolerance_above()) {
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
  const double dx = pose.position().x() - c.target().pose().position().x();
  const double dy = pose.position().y() - c.target().pose().position().y();
  const double dz = pose.position().z() - c.target().pose().position().z();
  return std::sqrt(dx * dx + dy * dy + dz * dz) <= c.tolerance();
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
  const double dot = std::abs(
      pose.orientation().w() * c.target().pose().orientation().w() +
      pose.orientation().x() * c.target().pose().orientation().x() +
      pose.orientation().y() * c.target().pose().orientation().y() +
      pose.orientation().z() * c.target().pose().orientation().z());
  const double clamped = std::min(1.0, std::max(0.0, dot));
  const double angle = 2.0 * std::acos(clamped);
  return angle <= c.tolerance();
}

/**
 * @brief Whether FK tip of @p state satisfies all Cartesian path constraints.
 */
inline bool SatisfiesCartesianConstraints(const planning::MotionPlanRequest& req,
                                          const core::JointState& state) {
  if (req.position_constraints.empty() && req.orientation_constraints.empty()) {
    return true;
  }
  if (!req.kinematics) {
    return false;
  }
  kinematics::Pose tip;
  if (!req.kinematics->GetPositionFK(state, &tip)) {
    return false;
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
  return true;
}

/**
 * @brief Check that every waypoint satisfies joint and Cartesian path constraints.
 * @param[in] req Request carrying joint / position / orientation constraints.
 * @param[in] traj Trajectory to validate.
 * @return true if all waypoints are valid.
 */
inline bool SatisfiesPathConstraints(const planning::MotionPlanRequest& req,
                                     const core::RobotTrajectory& traj) {
  for (int i = 0; i < traj.points_size(); ++i) {
    const core::JointState wp = MakeJointStateFromPoint(traj, i);
    if (!SatisfiesJointConstraints(req, wp)) {
      return false;
    }
    if (!SatisfiesCartesianConstraints(req, wp)) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Clamp @p state into joint constraint bands ∩ model limits (Project lite).
 * @return true if every constrained DOF could be clamped into a non-empty band.
 */
inline bool ClampToJointConstraints(const planning::MotionPlanRequest& req,
                                    core::JointState* state) {
  if (!state || state->position_size() == 0) {
    return false;
  }
  for (int i = 0; i < state->position_size(); ++i) {
    double lo = -1e9;
    double hi = 1e9;
    bool constrained = false;
    const std::string name =
        i < state->name_size() ? state->name(i) : std::string();
    if (req.model && !name.empty()) {
      if (const auto* lim = req.model->GetJointLimits(name)) {
        if (lim->has_position_limits) {
          lo = lim->min_position;
          hi = lim->max_position;
          constrained = true;
        }
      }
    }
    for (const auto& c : req.joint_constraints) {
      if (c.joint_name() != name) {
        continue;
      }
      lo = std::max(lo, c.position() - c.tolerance_below());
      hi = std::min(hi, c.position() + c.tolerance_above());
      constrained = true;
    }
    if (!constrained) {
      continue;
    }
    if (hi < lo) {
      return false;
    }
    state->set_position(i, std::min(hi, std::max(lo, state->position(i))));
  }
  return SatisfiesJointConstraints(req, *state);
}

/**
 * @brief Project @p state onto Cartesian path constraints via IK (MoveIt lite).
 *
 * Blends the FK tip toward position/orientation targets when outside tolerance,
 * then solves IK from @p state as seed. Returns false if kinematics missing or
 * IK fails; leaves @p state unchanged on failure.
 */
inline bool ProjectOntoCartesianConstraints(
    const planning::MotionPlanRequest& req, core::JointState* state) {
  if (!state || !req.kinematics) {
    return false;
  }
  if (req.position_constraints.empty() && req.orientation_constraints.empty()) {
    return true;
  }
  kinematics::Pose tip;
  if (!req.kinematics->GetPositionFK(*state, &tip)) {
    return false;
  }
  kinematics::Pose target = tip;
  bool need_ik = false;

  for (const auto& c : req.position_constraints) {
    const double dx = tip.position().x() - c.target().pose().position().x();
    const double dy = tip.position().y() - c.target().pose().position().y();
    const double dz = tip.position().z() - c.target().pose().position().z();
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist <= c.tolerance()) {
      continue;
    }
    need_ik = true;
    // Pull onto the constraint ball surface (toward target).
    const double scale =
        c.tolerance() > 1e-9 ? (c.tolerance() / dist) : 0.0;
    target.mutable_position()->set_x(c.target().pose().position().x() + dx * scale);
    target.mutable_position()->set_y(c.target().pose().position().y() + dy * scale);
    target.mutable_position()->set_z(c.target().pose().position().z() + dz * scale);
  }

  for (const auto& c : req.orientation_constraints) {
    const double dot =
        std::abs(tip.orientation().w() * c.target().pose().orientation().w() +
                 tip.orientation().x() * c.target().pose().orientation().x() +
                 tip.orientation().y() * c.target().pose().orientation().y() +
                 tip.orientation().z() * c.target().pose().orientation().z());
    const double clamped = std::min(1.0, std::max(0.0, dot));
    const double angle = 2.0 * std::acos(clamped);
    if (angle <= c.tolerance()) {
      continue;
    }
    need_ik = true;
    // Slerp tip → target with fraction so remaining angle ≈ tolerance.
    const double frac =
        angle > 1e-9 ? std::max(0.0, 1.0 - c.tolerance() / angle) : 1.0;
    const double a = 1.0 - frac;
    const double b = frac;
    // Match quaternion hemisphere.
    const double s =
        (tip.orientation().w() * c.target().pose().orientation().w() +
         tip.orientation().x() * c.target().pose().orientation().x() +
         tip.orientation().y() * c.target().pose().orientation().y() +
         tip.orientation().z() * c.target().pose().orientation().z()) < 0.0
            ? -1.0
            : 1.0;
    double qw = a * tip.orientation().w() + b * s * c.target().pose().orientation().w();
    double qx = a * tip.orientation().x() + b * s * c.target().pose().orientation().x();
    double qy = a * tip.orientation().y() + b * s * c.target().pose().orientation().y();
    double qz = a * tip.orientation().z() + b * s * c.target().pose().orientation().z();
    const double n = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    if (n > 1e-12) {
      qw /= n;
      qx /= n;
      qy /= n;
      qz /= n;
    }
    target.mutable_orientation()->set_w(qw);
    target.mutable_orientation()->set_x(qx);
    target.mutable_orientation()->set_y(qy);
    target.mutable_orientation()->set_z(qz);
  }

  if (!need_ik) {
    return true;
  }

  kinematics::IkOptions opts;
  opts.timeout = 0.05;
  opts.max_attempts = 8;
  opts.position_only = req.orientation_constraints.empty();
  core::JointState sol;
  if (req.kinematics->SearchPositionIK(target, *state, opts, &sol) !=
      ErrorCode::kSuccess) {
    return false;
  }
  *state = std::move(sol);
  return SatisfiesJointConstraints(req, *state) &&
         SatisfiesCartesianConstraints(req, *state);
}

/**
 * @brief Joint-space sample within joint constraints ∩ model limits (MoveIt
 * JointConstraintSampler lite).
 *
 * Unconstrained DOFs are drawn uniformly in model limits (or ±π fallback).
 * When @p near_seed is set, samples are biased toward the current @p state
 * (MoveIt near-seed sampling lite).
 */
inline bool SampleJointConstrainedState(const planning::MotionPlanRequest& req,
                                        core::JointState* state,
                                        std::mt19937* rng,
                                        bool near_seed = false) {
  if (!state || !rng || state->position_size() == 0) {
    return false;
  }
  for (int i = 0; i < state->position_size(); ++i) {
    double lo = -3.141592653589793;
    double hi = 3.141592653589793;
    const std::string name =
        i < state->name_size() ? state->name(i) : std::string();
    if (req.model && !name.empty()) {
      if (const auto* lim = req.model->GetJointLimits(name)) {
        if (lim->has_position_limits) {
          lo = lim->min_position;
          hi = lim->max_position;
        } else {
          lo = -1e3;
          hi = 1e3;
        }
      }
    }
    for (const auto& c : req.joint_constraints) {
      if (c.joint_name() != name) {
        continue;
      }
      lo = std::max(lo, c.position() - c.tolerance_below());
      hi = std::min(hi, c.position() + c.tolerance_above());
    }
    if (hi < lo) {
      return false;
    }
    if (near_seed) {
      const double center = state->position(i);
      const double span = 0.25 * (hi - lo);
      const double nlo = std::max(lo, center - span);
      const double nhi = std::min(hi, center + span);
      if (nhi >= nlo) {
        lo = nlo;
        hi = nhi;
      }
    }
    std::uniform_real_distribution<double> dist(lo, hi);
    state->set_position(i, dist(*rng));
  }
  return SatisfiesJointConstraints(req, *state);
}

/**
 * @brief Sample a tip pose inside position/orientation constraint regions.
 */
inline bool SampleConstrainedTipPose(const planning::MotionPlanRequest& req,
                                     kinematics::Pose* pose, std::mt19937* rng) {
  if (!pose || !rng) {
    return false;
  }
  *pose = MakeIdentityPose();
  if (!req.position_constraints.empty()) {
    const auto& c = req.position_constraints.front();
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    // Rejection sample in ball of radius tolerance.
    for (int k = 0; k < 32; ++k) {
      const double dx = u(*rng);
      const double dy = u(*rng);
      const double dz = u(*rng);
      const double n2 = dx * dx + dy * dy + dz * dz;
      if (n2 > 1.0 || n2 < 1e-12) {
        continue;
      }
      const double n = std::sqrt(n2);
      std::uniform_real_distribution<double> rdist(0.0, c.tolerance());
      const double r = rdist(*rng);
      pose->mutable_position()->set_x(c.target().pose().position().x() + r * dx / n);
      pose->mutable_position()->set_y(c.target().pose().position().y() + r * dy / n);
      pose->mutable_position()->set_z(c.target().pose().position().z() + r * dz / n);
      break;
    }
  }
  if (!req.orientation_constraints.empty()) {
    const auto& c = req.orientation_constraints.front();
    double qw = c.target().pose().orientation().w();
    double qx = c.target().pose().orientation().x();
    double qy = c.target().pose().orientation().y();
    double qz = c.target().pose().orientation().z();
    // Small random tilt within tolerance (axis-angle approx).
    std::uniform_real_distribution<double> ud(-1.0, 1.0);
    std::uniform_real_distribution<double> ad(0.0, c.tolerance());
    const double ax = ud(*rng);
    const double ay = ud(*rng);
    const double az = ud(*rng);
    const double an = std::sqrt(ax * ax + ay * ay + az * az);
    if (an > 1e-9) {
      const double ang = ad(*rng);
      const double s = std::sin(ang * 0.5);
      const double w = std::cos(ang * 0.5);
      const double x = s * ax / an;
      const double y = s * ay / an;
      const double z = s * az / an;
      // q = dq ⊗ target
      const double tw = c.target().pose().orientation().w();
      const double tx = c.target().pose().orientation().x();
      const double ty = c.target().pose().orientation().y();
      const double tz = c.target().pose().orientation().z();
      qw = w * tw - x * tx - y * ty - z * tz;
      qx = w * tx + x * tw + y * tz - z * ty;
      qy = w * ty - x * tz + y * tw + z * tx;
      qz = w * tz + x * ty - y * tx + z * tw;
    }
    pose->mutable_orientation()->set_w(qw);
    pose->mutable_orientation()->set_x(qx);
    pose->mutable_orientation()->set_y(qy);
    pose->mutable_orientation()->set_z(qz);
  } else if (req.position_constraints.empty()) {
    return false;
  }
  return true;
}

/**
 * @brief IKConstraintSampler lite: sample tip in constraint region → IK.
 *
 * @param[in] seed Seed / template joint state (names + DOF).
 * @param[in] max_attempts Number of tip/IK attempts.
 */
inline bool SampleIkConstrainedState(const planning::MotionPlanRequest& req,
                                     const core::JointState& seed,
                                     core::JointState* state, std::mt19937* rng,
                                     int max_attempts = 16) {
  if (!state || !rng || !req.kinematics || seed.position_size() == 0) {
    return false;
  }
  if (req.position_constraints.empty() && req.orientation_constraints.empty()) {
    *state = seed;
    return SampleJointConstrainedState(req, state, rng);
  }
  kinematics::IkOptions opts;
  opts.max_attempts = 8;
  opts.timeout = 0.05;
  opts.position_only = req.orientation_constraints.empty();
  for (int a = 0; a < max_attempts; ++a) {
    kinematics::Pose tip;
    if (!SampleConstrainedTipPose(req, &tip, rng)) {
      return false;
    }
    core::JointState sol;
    if (req.kinematics->SearchPositionIK(tip, seed, opts, &sol) !=
        ErrorCode::kSuccess) {
      continue;
    }
    if (!SatisfiesJointConstraints(req, sol)) {
      continue;
    }
    if (!SatisfiesCartesianConstraints(req, sol)) {
      continue;
    }
    if (req.scene && !req.scene->IsStateValid(sol)) {
      continue;
    }
    *state = std::move(sol);
    return true;
  }
  return false;
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
  if (req.goal_state.position_size() > 0 &&
      !SatisfiesJointConstraints(req, req.goal_state)) {
    if (error) {
      *error = "goal state violates joint constraints";
    }
    return ErrorCode::kGoalViolatesPathConstraints;
  }
  if (!req.position_constraints.empty() ||
      !req.orientation_constraints.empty()) {
    if (!req.kinematics) {
      if (error) {
        *error = "Cartesian constraints require kinematics";
      }
      return ErrorCode::kFailure;
    }
    if (!SatisfiesCartesianConstraints(req, req.start_state)) {
      if (error) {
        *error = "start state violates Cartesian path constraints";
      }
      return ErrorCode::kStartStateViolatesPathConstraints;
    }
    if (req.goal_state.position_size() > 0 &&
        !SatisfiesCartesianConstraints(req, req.goal_state)) {
      if (error) {
        *error = "goal state violates Cartesian path constraints";
      }
      return ErrorCode::kGoalViolatesPathConstraints;
    }
  }
  return ErrorCode::kSuccess;
}

}  // namespace constraint_samplers
}  // namespace manipulation
}  // namespace autonomy
