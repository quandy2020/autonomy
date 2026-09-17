/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/planner/joint_interpolation/joint_interpolation_planner.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/common/joint_state_util.hpp"
#include "autonomy/manipulation/model/error_codes.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"
#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {
namespace {

bool SameDof(const core::JointState& a, const core::JointState& b) {
  return a.position_size() == b.position_size() && a.position_size() > 0;
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
  const int dof = GetJointStateDof(request.start_state);
  std::vector<std::string> names;
  if (request.goal_state.name_size() > 0) {
    names.assign(request.goal_state.name().begin(),
                 request.goal_state.name().end());
  } else {
    names.assign(request.start_state.name().begin(),
                 request.start_state.name().end());
  }

  response.trajectory.Clear();
  for (int i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n - 1);
    std::vector<double> positions(static_cast<std::size_t>(dof));
    for (int j = 0; j < dof; ++j) {
      const double q0 = request.start_state.position(j);
      const double q1 = request.goal_state.position(j);
      positions[static_cast<std::size_t>(j)] = q0 + t * (q1 - q0);
    }
    core::JointState waypoint;
    SetJointState(&waypoint, names, positions);
    AddTrajectoryPoint(&response.trajectory, waypoint, dt * static_cast<double>(i));
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
