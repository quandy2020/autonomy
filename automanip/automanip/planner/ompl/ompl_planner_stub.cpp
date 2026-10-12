/*
 * Copyright 2026 The Openbot Authors
 *
 * OMPL planner — stub backend when AUTOMANIP_HAS_OMPL is off.
 */

#include "automanip/planner/ompl/ompl_planner.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/common/logging.hpp"
#include "automanip/model/error_codes.hpp"

namespace automanip {
namespace planner {

#ifndef AUTOMANIP_HAS_OMPL

bool OmplPlanner::Init(const std::string& planner_id) {
  planner_id_ = planner_id;
  AWARN << "OmplPlanner: AUTOMANIP_HAS_OMPL off; install OMPL to enable";
  return true;
}

::automanip::proto::MotionPlanResponse OmplPlanner::Plan(const MotionPlanRequest& /*request*/) {
  ::automanip::proto::MotionPlanResponse response;
  response.set_error_code(ErrorCode::PLANNING_FAILED);
  response.set_error("ompl backend not linked (AUTOMANIP_HAS_OMPL off)");
  return response;
}

common::PlannerInterface::SharedPtr CreateOmplPlanner() {
  return std::make_shared<OmplPlanner>();
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(OmplPlanner, common::PlannerInterface);

#endif  // !AUTOMANIP_HAS_OMPL

}  // namespace planner
}  // namespace automanip
