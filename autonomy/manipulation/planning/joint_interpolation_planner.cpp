/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/planning/joint_interpolation_planner.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/core/error_codes.hpp"
#include "autonomy/manipulation/scene/planning_scene.hpp"
#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {
namespace {

bool SameDof(const core::JointState& a, const core::JointState& b) {
  return a.positions.size() == b.positions.size() && !a.positions.empty();
}

}  // namespace

bool JointInterpolationPlanner::Init(const std::string& planner_id) {
  planner_id_ = planner_id;
  AINFO << "JointInterpolationPlanner init id=" << planner_id_;
  return true;
}

MotionPlanResponse JointInterpolationPlanner::Plan(
    const MotionPlanRequest& request) {
  MotionPlanResponse response;
  if (request.has_goal_pose) {
    response.error = "pose goal requires IK; use joint goal";
    response.error_code = ErrorCode::kInvalidGoalConstraints;
    return response;
  }
  if (!SameDof(request.start_state, request.goal_state)) {
    response.error = "start/goal DOF mismatch or empty";
    response.error_code = ErrorCode::kInvalidRobotState;
    return response;
  }

  if (request.scene && request.scene->CheckCollision(request.start_state)) {
    response.error = "start in collision";
    response.error_code = ErrorCode::kStartStateInCollision;
    return response;
  }
  if (request.scene && request.scene->CheckCollision(request.goal_state)) {
    response.error = "goal in collision";
    response.error_code = ErrorCode::kGoalInCollision;
    return response;
  }

  const int n = std::max(2, num_steps_);
  const double dt =
      request.planning_time > 0.0
          ? request.planning_time / static_cast<double>(n - 1)
          : 0.05;
  const std::size_t dof = request.start_state.positions.size();

  response.trajectory.waypoints.clear();
  response.trajectory.time_from_start.clear();
  response.trajectory.waypoints.reserve(static_cast<std::size_t>(n));
  response.trajectory.time_from_start.reserve(static_cast<std::size_t>(n));

  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n - 1);
    core::JointState waypoint;
    waypoint.names = request.goal_state.names.empty()
                         ? request.start_state.names
                         : request.goal_state.names;
    waypoint.positions.resize(dof);
    for (std::size_t j = 0; j < dof; ++j) {
      const double q0 = request.start_state.positions[j];
      const double q1 = request.goal_state.positions[j];
      waypoint.positions[j] = q0 + t * (q1 - q0);
    }
    response.trajectory.waypoints.push_back(std::move(waypoint));
    response.trajectory.time_from_start.push_back(dt * static_cast<double>(i));
  }

  if (request.scene && !request.scene->IsPathValid(response.trajectory)) {
    response.error = "interpolated path in collision";
    response.error_code = ErrorCode::kInvalidMotionPlan;
    response.trajectory = {};
    return response;
  }

  response.success = true;
  response.error_code = ErrorCode::kSuccess;
  return response;
}


AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(JointInterpolationPlanner, PlannerBase);

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
