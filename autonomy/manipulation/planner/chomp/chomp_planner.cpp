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
#include "autonomy/manipulation/planner/optimize/trajectory_optimize.hpp"
#include "autonomy/manipulation/common/joint_state_util.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {

bool ChompPlanner::Init(const std::string& planner_id) {
  planner_id_ = planner_id;
  return true;
}

MotionPlanResponse ChompPlanner::Plan(const MotionPlanRequest& request) {
  MotionPlanResponse response;
  if (GetJointStateDof(request.start_state) !=
          GetJointStateDof(request.goal_state) ||
      request.start_state.position_size() == 0) {
    response.error_code = ErrorCode::kInvalidRobotState;
    response.error = "CHOMP DOF mismatch";
    return response;
  }
  ChompParams params = DefaultChompParams();
  LoadChompParamsFromShare(&params);
  if (request.planning_time > 0) {
    params.max_iterations =
        std::max(10, static_cast<int>(request.planning_time * 20));
    if (params.planning_time_limit <= 0.0) {
      params.planning_time_limit = request.planning_time;
    }
  }
  core::RobotTrajectory traj = optimize::InterpolateSeedTrajectory(
      request, params.num_timesteps, params.trajectory_initialization_method);
  traj = optimize::ChompOptimize(request, std::move(traj), params);
  int recovery = 0;
  while (request.scene && !request.scene->IsPathValid(traj) &&
         params.enable_failure_recovery &&
         recovery < params.max_recovery_attempts) {
    ++recovery;
    std::mt19937 rng(42 + recovery * 17);
    auto seed = optimize::InterpolateSeedTrajectory(
        request, params.num_timesteps, params.trajectory_initialization_method);
    seed = optimize::PerturbSeed(seed, &rng);
    optimize::ClampTrajectory(request, &seed);
    traj = optimize::ChompOptimize(request, std::move(seed), params);
  }
  if (request.scene && !request.scene->IsPathValid(traj)) {
    response.error_code = ErrorCode::kPlanningFailed;
    response.error = "CHOMP failed to clear collision";
    return response;
  }
  response.trajectory = std::move(traj);
  response.success = true;
  response.error_code = ErrorCode::kSuccess;
  return response;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(ChompPlanner, PlannerBase);

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
