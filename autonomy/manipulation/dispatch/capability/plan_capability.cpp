/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/dispatch/capability/plan_capability.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/manipulation_server.hpp"

namespace autonomy {
namespace manipulation {
namespace dispatch {

bool PlanCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

::autonomy::manipulation::proto::MotionPlanResponse PlanCapability::Plan(
    const planner::MotionPlanRequest& req) {
  return server_->Plan(req);
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(PlanCapability, Capability);

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
