/*
 * Copyright 2026 The Openbot Authors
 *
 * OMPL planner — stub backend when AUTONOMY_HAS_OMPL is off.
 */

#include "autonomy/manipulation/planner/ompl/ompl_planner.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {

#ifndef AUTONOMY_HAS_OMPL

bool OmplPlanner::Init(const std::string& planner_id) {
  planner_id_ = planner_id;
  AWARN << "OmplPlanner: AUTONOMY_HAS_OMPL off; install OMPL to enable";
  return true;
}

::autonomy::manipulation::proto::MotionPlanResponse OmplPlanner::Plan(const MotionPlanRequest& /*request*/) {
  ::autonomy::manipulation::proto::MotionPlanResponse response;
  response.set_error_code(ErrorCode::PLANNING_FAILED);
  response.set_error("ompl backend not linked (AUTONOMY_HAS_OMPL off)");
  return response;
}

PlannerInterface::SharedPtr CreateOmplPlanner() {
  return std::make_shared<OmplPlanner>();
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(OmplPlanner, PlannerInterface);

#endif  // !AUTONOMY_HAS_OMPL

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
