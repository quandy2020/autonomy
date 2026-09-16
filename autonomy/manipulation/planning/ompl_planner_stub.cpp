/*
 * Copyright 2026 The Openbot Authors
 *
 * OMPL planner — stub backend when AUTONOMY_HAS_OMPL is off.
 */

#include "autonomy/manipulation/planning/ompl_planner.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {

#ifndef AUTONOMY_HAS_OMPL

bool OmplPlanner::Init(const std::string& planner_id) {
  planner_id_ = planner_id;
  AWARN << "OmplPlanner: AUTONOMY_HAS_OMPL off; install OMPL to enable";
  return true;
}

MotionPlanResponse OmplPlanner::Plan(const MotionPlanRequest& /*request*/) {
  MotionPlanResponse response;
  response.error_code = ErrorCode::kPlanningFailed;
  response.error = "ompl backend not linked (AUTONOMY_HAS_OMPL off)";
  return response;
}

std::shared_ptr<PlannerBase> CreateOmplPlanner() {
  return std::make_shared<OmplPlanner>();
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(OmplPlanner, PlannerBase);

#endif  // !AUTONOMY_HAS_OMPL

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
