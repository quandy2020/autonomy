/*
 * Copyright 2026 The Openbot Authors
 *
 * Sampling seed (OMPL / RRT-Connect) + CHOMP polish hybrid planner.
 */

#include "autonomy/manipulation/planner/hybrid/hybrid_planner.hpp"

#include <algorithm>
#include <memory>
#include <utility>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/common/plugin_ids.hpp"
#include "autonomy/manipulation/planner/optimize/trajectory_optimize.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {

bool HybridPlanner::Init(const std::string& planner_id) {
  planner_id_ = planner_id;
  return true;
}

MotionPlanResponse HybridPlanner::Plan(const MotionPlanRequest& request) {
  MotionPlanResponse response;
  RegisterManipulationPlugins();

  auto try_seed = [&](const char* id) -> MotionPlanResponse {
    MotionPlanResponse out;
    auto planner = CreatePlugin<PlannerBase>(id);
    if (!planner || !planner->Init(id)) {
      out.error_code = ErrorCode::kFailure;
      out.error = std::string("hybrid: seed init failed: ") + id;
      return out;
    }
    return planner->Plan(request);
  };

  response = try_seed("ompl");
  if (!response.success) {
    response = try_seed("rrt_connect");
  }
  if (!response.success || response.trajectory.points_size() < 3) {
    return response;
  }

  const int iters = std::max(8, static_cast<int>(request.planning_time * 10));
  auto polished =
      optimize::ChompPolish(request, response.trajectory, iters);
  if (!request.scene || request.scene->IsPathValid(polished)) {
    response.trajectory = std::move(polished);
  }
  response.success = true;
  response.error_code = ErrorCode::kSuccess;
  return response;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(HybridPlanner, PlannerBase);

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
