/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/planner/pipeline/planning_request_adapter.hpp"

#include <algorithm>
#include <cmath>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/common/joint_state_util.hpp"
#include "autonomy/manipulation/planner/constraint_samplers/constraint_samplers.hpp"
#include "autonomy/manipulation/planner/pipeline/time_parameterization.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {

bool ApplyTimeParameterizationAdapter::Adapt(MotionPlanRequest* request,
                                             MotionPlanResponse* response) const {
  if (!request || !response || !response->success) {
    return true;
  }
  trajectory::TimeParamOptions opts;
  opts.max_velocity = request->max_velocity;
  opts.max_acceleration = request->max_acceleration;
  opts.path_tolerance = std::max(1e-3, request->blend_radius);
  if (request->model && response->trajectory.points_size() > 0) {
    opts.max_velocity_vector.resize(
        static_cast<std::size_t>(response->trajectory.joint_names_size()));
    opts.max_acceleration_vector.resize(
        static_cast<std::size_t>(response->trajectory.joint_names_size()));
    for (int i = 0; i < response->trajectory.joint_names_size(); ++i) {
      const auto* lim =
          request->model->GetJointLimits(response->trajectory.joint_names(i));
      opts.max_velocity_vector[static_cast<std::size_t>(i)] =
          lim && lim->max_velocity > 0
              ? lim->max_velocity * std::max(1e-3, request->velocity_scale)
              : opts.max_velocity * std::max(1e-3, request->velocity_scale);
      opts.max_acceleration_vector[static_cast<std::size_t>(i)] =
          lim && lim->max_acceleration > 0 ? lim->max_acceleration
                                           : opts.max_acceleration;
    }
  }
#ifdef AUTONOMY_HAS_RUCKIG
  if (trajectory::ApplyRuckig(&response->trajectory, opts)) {
    return true;
  }
#endif
  return trajectory::ApplyTotg(&response->trajectory, opts);
}

bool ValidatePathAdapter::Adapt(MotionPlanRequest* request,
                                MotionPlanResponse* response) const {
  if (!request || !response || !response->success) {
    return true;
  }
  if (request->scene &&
      !request->scene->IsPathValidDense(response->trajectory, 4)) {
    response->success = false;
    response->error_code = ErrorCode::kInvalidMotionPlan;
    response->error = "adapter validate_path (dense) failed";
    response->trajectory = {};
    return false;
  }
  return true;
}

bool DenseSampleAdapter::Adapt(MotionPlanRequest* request,
                               MotionPlanResponse* response) const {
  if (!request || !response || !response->success ||
      response->trajectory.points_size() < 2) {
    return true;
  }
  core::RobotTrajectory dense;
  for (int i = 0; i + 1 < response->trajectory.points_size(); ++i) {
    const core::JointState a =
        MakeJointStateFromPoint(response->trajectory, i);
    const core::JointState b =
        MakeJointStateFromPoint(response->trajectory, i + 1);
    AddTrajectoryPoint(&dense, a, 0.0);
    double max_dq = 0.0;
    const int n = std::min(a.position_size(), b.position_size());
    for (int j = 0; j < n; ++j) {
      max_dq = std::max(max_dq, std::abs(b.position(j) - a.position(j)));
    }
    const int mid = static_cast<int>(std::ceil(max_dq / max_step_)) - 1;
    for (int k = 1; k <= mid; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(mid + 1);
      core::JointState wp = a;
      ResizeJointState(&wp, n);
      for (int j = 0; j < n; ++j) {
        wp.set_position(j, a.position(j) + t * (b.position(j) - a.position(j)));
      }
      AddTrajectoryPoint(&dense, wp, 0.0);
    }
  }
  AddTrajectoryPoint(
      &dense,
      MakeJointStateFromPoint(response->trajectory,
                              response->trajectory.points_size() - 1),
      0.0);
  for (int i = 0; i < dense.points_size(); ++i) {
    SetDurationSeconds(0.05 * static_cast<double>(i),
                       dense.mutable_points(i)->mutable_time_from_start());
  }
  response->trajectory = std::move(dense);
  return true;
}

bool CheckConstraintsAdapter::Adapt(MotionPlanRequest* request,
                                    MotionPlanResponse* response) const {
  if (!request || !response || !response->success) {
    return true;
  }
  if (!constraint_samplers::SatisfiesPathConstraints(request,
                                                     response->trajectory)) {
    response->success = false;
    response->error_code = ErrorCode::kGoalViolatesPathConstraints;
    response->error = "adapter check_constraints failed";
    response->trajectory = {};
    return false;
  }
  return true;
}

bool FixStartStateBoundsAdapter::Adapt(MotionPlanRequest* request,
                                       MotionPlanResponse* /*response*/) const {
  if (!request) {
    return true;
  }
  if (request->model) {
    for (int i = 0; i < request->start_state.name_size(); ++i) {
      if (i >= request->start_state.position_size()) {
        break;
      }
      const auto* lim =
          request->model->GetJointLimits(request->start_state.name(i));
      if (!lim || !lim->has_position_limits) {
        continue;
      }
      request->start_state.set_position(
          i, std::clamp(request->start_state.position(i), lim->min_position,
                        lim->max_position));
    }
  }
  if (!request->joint_constraints.empty()) {
    constraint_samplers::ClampToJointConstraints(*request,
                                                 &request->start_state);
  }
  return true;
}

bool FixStartStatePathConstraintsAdapter::Adapt(
    MotionPlanRequest* request, MotionPlanResponse* response) const {
  if (!request) {
    return true;
  }
  // Bounds + joint bands first.
  FixStartStateBoundsAdapter bounds;
  bounds.Adapt(request, response);

  const bool has_cart = !request->position_constraints.empty() ||
                        !request->orientation_constraints.empty();
  if (!has_cart && request->joint_constraints.empty()) {
    return true;
  }
  if (has_cart) {
    if (!constraint_samplers::ProjectOntoCartesianConstraints(
            *request, &request->start_state)) {
      if (response) {
        response->success = false;
        response->error_code = ErrorCode::kStartStateViolatesPathConstraints;
        response->error =
            "fix_start_state_path_constraints: Cartesian project failed";
      }
      return false;
    }
  } else if (!constraint_samplers::ClampToJointConstraints(
                 *request, &request->start_state)) {
    if (response) {
      response->success = false;
      response->error_code = ErrorCode::kStartStateViolatesPathConstraints;
      response->error =
          "fix_start_state_path_constraints: joint clamp failed";
    }
    return false;
  }
  return true;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(ApplyTimeParameterizationAdapter,
                                        PlanningRequestAdapter);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(ValidatePathAdapter,
                                        PlanningRequestAdapter);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(DenseSampleAdapter,
                                        PlanningRequestAdapter);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(CheckConstraintsAdapter,
                                        PlanningRequestAdapter);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(FixStartStateBoundsAdapter,
                                        PlanningRequestAdapter);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(FixStartStatePathConstraintsAdapter,
                                        PlanningRequestAdapter);

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
