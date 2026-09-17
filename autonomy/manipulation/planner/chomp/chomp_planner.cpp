/*
 * Copyright 2026 The Openbot Authors
 *
 * CHOMP-style covariant gradient trajectory optimizer (industrial-lite).
 */

#include "autonomy/manipulation/planner/chomp/chomp_planner.hpp"

#include <algorithm>
#include <random>
#include <utility>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/planner/chomp/chomp_params.hpp"
#include "autonomy/manipulation/utils/trajectory_optimize.hpp"
#include "autonomy/manipulation/model/error_codes.hpp"
#include "autonomy/manipulation/model/joint_state_utilities.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {

bool ChompPlanner::Init(const std::string& planner_id) {
  planner_id_ = planner_id;
  return true;
}

::autonomy::manipulation::proto::MotionPlanResponse ChompPlanner::Plan(const MotionPlanRequest& request) {
  ::autonomy::manipulation::proto::MotionPlanResponse response;
  if (GetJointStateDegreesOfFreedom(request.pb.start_state()) !=
          GetJointStateDegreesOfFreedom(request.pb.goal_state()) ||
      request.pb.start_state().position_size() == 0) {
    response.set_error_code(ErrorCode::INVALID_ROBOT_STATE);
    response.set_error("CHOMP DOF mismatch");
    return response;
  }
  ChompParams params = DefaultChompParams();
  LoadChompParamsFromShare(&params);
  if (request.pb.planning_time() > 0) {
    params.max_iterations =
        std::max(10, static_cast<int>(request.pb.planning_time() * 20));
    if (params.planning_time_limit <= 0.0) {
      params.planning_time_limit = request.pb.planning_time();
    }
  }
  automsgs::msgs::trajectory_msgs::JointTrajectory traj = trajopt::InterpolateSeedTrajectory(
      request, params.num_timesteps, params.trajectory_initialization_method);
  traj = trajopt::ChompOptimize(request, std::move(traj), params);
  int recovery = 0;
  while (request.scene && !request.scene->IsPathValid(traj) &&
         params.enable_failure_recovery &&
         recovery < params.max_recovery_attempts) {
    ++recovery;
    std::mt19937 rng(42 + recovery * 17);
    auto seed = trajopt::InterpolateSeedTrajectory(
        request, params.num_timesteps, params.trajectory_initialization_method);
    seed = trajopt::PerturbSeed(seed, &rng);
    trajopt::ClampTrajectory(request, &seed);
    traj = trajopt::ChompOptimize(request, std::move(seed), params);
  }
  if (request.scene && !request.scene->IsPathValid(traj)) {
    response.set_error_code(ErrorCode::PLANNING_FAILED);
    response.set_error("CHOMP failed to clear collision");
    return response;
  }
  *response.mutable_trajectory() = std::move(traj);
  response.set_success(true);
  response.set_error_code(ErrorCode::SUCCESS);
  return response;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(ChompPlanner, common::PlannerInterface);

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
