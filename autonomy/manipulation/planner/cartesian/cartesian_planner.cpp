/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/planner/cartesian/cartesian_planner.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/common/joint_state_util.hpp"
#include "autonomy/manipulation/motion/kinematics/pose_util.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"
#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {
namespace {

using kinematics::Pose;
using kinematics::InterpolatePose;

}  // namespace

bool CartesianPlanner::Init(const std::string& planner_id) {
  planner_id_ = planner_id;
  return true;
}

MotionPlanResponse CartesianPlanner::Plan(const MotionPlanRequest& request) {
  MotionPlanResponse response;
  if (!request.kinematics) {
    response.error = "cartesian planner requires kinematics";
    response.error_code = ErrorCode::kFailure;
    return response;
  }

  std::vector<kinematics::Pose> waypoints = request.cartesian_waypoints;
  if (waypoints.empty() && request.has_goal_pose) {
    kinematics::Pose start_pose;
    if (!request.kinematics->GetPositionFK(request.start_state, &start_pose)) {
      response.error = "FK failed at start";
      response.error_code = ErrorCode::kFailure;
      return response;
    }
    waypoints = {start_pose, request.goal_pose};
  }
  if (waypoints.size() < 2) {
    response.error = "need >=2 cartesian waypoints";
    response.error_code = ErrorCode::kInvalidGoalConstraints;
    return response;
  }

  const int steps = std::max(2, num_steps_);
  core::JointState seed = request.start_state;
  response.trajectory.Clear();
  AddTrajectoryPoint(&response.trajectory, seed, 0.0);

  for (std::size_t seg = 0; seg + 1 < waypoints.size(); ++seg) {
    for (int i = 1; i < steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(steps - 1);
      const auto pose = kinematics::InterpolatePose(waypoints[seg],
                                                     waypoints[seg + 1], t);
      core::JointState sol;
      kinematics::IkOptions opts;
      opts.position_only = false;
      opts.max_attempts = 4;
      if (request.kinematics->GetPositionIK(pose, seed, opts, &sol) !=
          ErrorCode::kSuccess) {
        response.error = "IK failed on cartesian path";
        response.error_code = ErrorCode::kNoIkSolution;
        response.trajectory = {};
        return response;
      }
      if (request.scene && request.scene->CheckCollision(sol)) {
        response.error = "cartesian path in collision";
        response.error_code = ErrorCode::kInvalidMotionPlan;
        response.trajectory = {};
        return response;
      }
      seed = sol;
      AddTrajectoryPoint(&response.trajectory, sol,
                         static_cast<double>(response.trajectory.points_size()) *
                             0.05);
    }
  }

  response.success = true;
  response.error_code = ErrorCode::kSuccess;
  return response;
}


AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(CartesianPlanner, PlannerBase);

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
