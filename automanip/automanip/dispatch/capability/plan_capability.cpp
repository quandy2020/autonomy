/*
 * Copyright 2026 The Openbot Authors
 */

#include "automanip/dispatch/capability/plan_capability.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "automanip/manipulation_server.hpp"

namespace automanip {
namespace dispatch {

bool PlanCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

::automanip::proto::MotionPlanResponse PlanCapability::Plan(
    const planner::MotionPlanRequest& req) {
  return server_->Plan(req);
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(PlanCapability, Capability);

}  // namespace dispatch
}  // namespace automanip
