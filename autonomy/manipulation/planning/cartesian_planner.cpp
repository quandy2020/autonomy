/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/planning/cartesian_planner.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/scene/planning_scene.hpp"
#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {
namespace {

using kinematics::Pose;

Pose LerpPose(const Pose& a, const Pose& b, double t) {
  Pose p;
  p.x = a.x + t * (b.x - a.x);
  p.y = a.y + t * (b.y - a.y);
  p.z = a.z + t * (b.z - a.z);
  // Nlerp orientation
  p.qx = a.qx + t * (b.qx - a.qx);
  p.qy = a.qy + t * (b.qy - a.qy);
  p.qz = a.qz + t * (b.qz - a.qz);
  p.qw = a.qw + t * (b.qw - a.qw);
  const double n =
      std::sqrt(p.qx * p.qx + p.qy * p.qy + p.qz * p.qz + p.qw * p.qw);
  if (n > 1e-9) {
    p.qx /= n;
    p.qy /= n;
    p.qz /= n;
    p.qw /= n;
  }
  return p;
}

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
  response.trajectory.waypoints.push_back(seed);
  response.trajectory.time_from_start.push_back(0.0);

  for (std::size_t seg = 0; seg + 1 < waypoints.size(); ++seg) {
    for (int i = 1; i < steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(steps - 1);
      const auto pose = LerpPose(waypoints[seg], waypoints[seg + 1], t);
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
      response.trajectory.waypoints.push_back(sol);
      response.trajectory.time_from_start.push_back(
          static_cast<double>(response.trajectory.waypoints.size() - 1) * 0.05);
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
