/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/dispatch/capability/plan_and_execute_capability.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/model/error_codes.hpp"
#include "autonomy/manipulation/manipulation_server.hpp"

namespace autonomy {
namespace manipulation {
namespace dispatch {

bool PlanAndExecuteCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

::autonomy::manipulation::proto::MotionPlanResponse PlanAndExecuteCapability::Run(
    const planner::MotionPlanRequest& req) {
  auto response = server_->Plan(req);
  if (!response.success()) {
    return response;
  }
  const ErrorCode code = server_->ExecuteTrajectory(response.trajectory());
  if (code != ErrorCode::SUCCESS) {
    response.set_success(false);
    response.set_error_code(code);
    response.set_error(ErrorCodeName(code));
  }
  return response;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(PlanAndExecuteCapability, Capability);

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
