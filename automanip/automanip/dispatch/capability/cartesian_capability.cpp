/*
 * Copyright 2026 The Openbot Authors
 */

#include "automanip/dispatch/capability/cartesian_capability.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "automanip/manipulation_server.hpp"

namespace automanip {
namespace dispatch {

bool CartesianCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

::automanip::proto::MotionPlanResponse CartesianCapability::Plan(
    const planner::MotionPlanRequest& req) {
  planner::MotionPlanRequest r = req;
  r.pb.set_planner_id("pilz_lin");
  return server_->Plan(r);
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(CartesianCapability, Capability);

}  // namespace dispatch
}  // namespace automanip
