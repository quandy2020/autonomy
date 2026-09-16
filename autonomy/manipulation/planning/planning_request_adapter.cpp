/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/planning/planning_request_adapter.hpp"

#include <algorithm>
#include <cmath>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/planning/constraint_samplers.hpp"
#include "autonomy/manipulation/planning/time_parameterization.hpp"
#include "autonomy/manipulation/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {

bool TimeParameterizeAdapter::Adapt(MotionPlanRequest* request,
                                    MotionPlanResponse* response) const {
  if (!request || !response || !response->success) {
    return true;
  }
  trajectory::TimeParamOptions opts;
  opts.max_velocity = request->max_velocity;
  opts.max_acceleration = request->max_acceleration;
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
  if (request->scene && !request->scene->IsPathValid(response->trajectory)) {
    response->success = false;
    response->error_code = ErrorCode::kInvalidMotionPlan;
    response->error = "adapter validate_path failed";
    response->trajectory = {};
    return false;
  }
  return true;
}

bool DenseSampleAdapter::Adapt(MotionPlanRequest* request,
                               MotionPlanResponse* response) const {
  if (!request || !response || !response->success ||
      response->trajectory.waypoints.size() < 2) {
    return true;
  }
  core::RobotTrajectory dense;
  for (std::size_t i = 0; i + 1 < response->trajectory.waypoints.size(); ++i) {
    const auto& a = response->trajectory.waypoints[i];
    const auto& b = response->trajectory.waypoints[i + 1];
    dense.waypoints.push_back(a);
    double max_dq = 0.0;
    const std::size_t n = std::min(a.positions.size(), b.positions.size());
    for (std::size_t j = 0; j < n; ++j) {
      max_dq = std::max(max_dq, std::abs(b.positions[j] - a.positions[j]));
    }
    const int mid = static_cast<int>(std::ceil(max_dq / max_step_)) - 1;
    for (int k = 1; k <= mid; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(mid + 1);
      core::JointState wp = a;
      wp.positions.resize(n);
      for (std::size_t j = 0; j < n; ++j) {
        wp.positions[j] =
            a.positions[j] + t * (b.positions[j] - a.positions[j]);
      }
      dense.waypoints.push_back(wp);
    }
  }
  dense.waypoints.push_back(response->trajectory.waypoints.back());
  dense.time_from_start.resize(dense.waypoints.size());
  for (std::size_t i = 0; i < dense.waypoints.size(); ++i) {
    dense.time_from_start[i] = 0.05 * static_cast<double>(i);
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
  if (!request || !request->model) {
    return true;
  }
  for (std::size_t i = 0; i < request->start_state.names.size(); ++i) {
    if (i >= request->start_state.positions.size()) {
      break;
    }
    const auto* lim =
        request->model->GetJointLimits(request->start_state.names[i]);
    if (!lim || !lim->has_position_limits) {
      continue;
    }
    request->start_state.positions[i] = std::clamp(
        request->start_state.positions[i], lim->min_position, lim->max_position);
  }
  return true;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(TimeParameterizeAdapter,
                                        PlanningRequestAdapter);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(ValidatePathAdapter,
                                        PlanningRequestAdapter);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(DenseSampleAdapter,
                                        PlanningRequestAdapter);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(CheckConstraintsAdapter,
                                        PlanningRequestAdapter);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(FixStartStateBoundsAdapter,
                                        PlanningRequestAdapter);

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
