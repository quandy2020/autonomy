/*
 * Copyright 2026 The Openbot Authors
 *
 * STOMP-style correlated-noise trajectory optimizer (industrial-lite).
 */

#include "autonomy/manipulation/planner/stomp/stomp_planner.hpp"

#include <algorithm>
#include <random>
#include <utility>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/planner/optimize/trajectory_optimize.hpp"
#include "autonomy/manipulation/common/joint_state_util.hpp"
#include "autonomy/manipulation/planner/stomp/stomp_params.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {

bool StompPlanner::Init(const std::string& planner_id) {
  planner_id_ = planner_id;
  return true;
}

MotionPlanResponse StompPlanner::Plan(const MotionPlanRequest& request) {
  MotionPlanResponse response;
  if (GetJointStateDof(request.start_state) !=
          GetJointStateDof(request.goal_state) ||
      request.start_state.position_size() == 0) {
    response.error_code = ErrorCode::kInvalidRobotState;
    response.error = "STOMP DOF mismatch";
    return response;
  }
  StompParams params = DefaultStompParams();
  LoadStompParamsFromShare(&params);
  if (request.planning_time > 0) {
    params.num_iterations =
        std::max(4, static_cast<int>(request.planning_time * 4));
    if (params.planning_time_limit <= 0.0) {
      params.planning_time_limit = request.planning_time;
    }
  }
  core::RobotTrajectory seed =
      optimize::InterpolateSeedTrajectory(request, params.num_timesteps);
  auto best = optimize::StompOptimize(request, std::move(seed), params);
  int recovery = 0;
  while (request.scene && !request.scene->IsPathValid(best) &&
         params.enable_failure_recovery &&
         recovery < params.max_recovery_attempts) {
    ++recovery;
    std::mt19937 rng(91 + recovery * 13);
    auto retry = optimize::InterpolateSeedTrajectory(request, params.num_timesteps);
    retry = optimize::PerturbSeed(retry, &rng);
    optimize::ClampTrajectory(request, &retry);
    best = optimize::StompOptimize(request, std::move(retry), params);
  }
  if (request.scene && !request.scene->IsPathValid(best)) {
    response.error_code = ErrorCode::kPlanningFailed;
    response.error = "STOMP no collision-free trajectory";
    return response;
  }
  response.trajectory = std::move(best);
  response.success = true;
  response.error_code = ErrorCode::kSuccess;
  return response;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(StompPlanner, PlannerBase);

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
