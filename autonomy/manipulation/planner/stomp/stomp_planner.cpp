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
#include "autonomy/manipulation/utils/trajectory_optimize.hpp"
#include "autonomy/manipulation/model/error_codes.hpp"
#include "autonomy/manipulation/model/joint_state_utilities.hpp"
#include "autonomy/manipulation/planner/stomp/stomp_params.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {

bool StompPlanner::Init(const std::string& planner_id) {
  planner_id_ = planner_id;
  return true;
}

::autonomy::manipulation::proto::MotionPlanResponse StompPlanner::Plan(const MotionPlanRequest& request) {
  ::autonomy::manipulation::proto::MotionPlanResponse response;
  if (GetJointStateDegreesOfFreedom(request.pb.start_state()) !=
          GetJointStateDegreesOfFreedom(request.pb.goal_state()) ||
      request.pb.start_state().position_size() == 0) {
    response.set_error_code(ErrorCode::INVALID_ROBOT_STATE);
    response.set_error("STOMP DOF mismatch");
    return response;
  }
  StompParams params = DefaultStompParams();
  LoadStompParamsFromShare(&params);
  if (request.pb.planning_time() > 0) {
    params.num_iterations =
        std::max(4, static_cast<int>(request.pb.planning_time() * 4));
    if (params.planning_time_limit <= 0.0) {
      params.planning_time_limit = request.pb.planning_time();
    }
  }
  automsgs::msgs::trajectory_msgs::JointTrajectory seed =
      trajopt::InterpolateSeedTrajectory(request, params.num_timesteps);
  auto best = trajopt::StompOptimize(request, std::move(seed), params);
  int recovery = 0;
  while (request.scene && !request.scene->IsPathValid(best) &&
         params.enable_failure_recovery &&
         recovery < params.max_recovery_attempts) {
    ++recovery;
    std::mt19937 rng(91 + recovery * 13);
    auto retry = trajopt::InterpolateSeedTrajectory(request, params.num_timesteps);
    retry = trajopt::PerturbSeed(retry, &rng);
    trajopt::ClampTrajectory(request, &retry);
    best = trajopt::StompOptimize(request, std::move(retry), params);
  }
  if (request.scene && !request.scene->IsPathValid(best)) {
    response.set_error_code(ErrorCode::PLANNING_FAILED);
    response.set_error("STOMP no collision-free trajectory");
    return response;
  }
  *response.mutable_trajectory() = std::move(best);
  response.set_success(true);
  response.set_error_code(ErrorCode::SUCCESS);
  return response;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(StompPlanner, common::PlannerInterface);

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
