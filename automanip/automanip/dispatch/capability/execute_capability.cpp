/*
 * Copyright 2026 The Openbot Authors
 */

#include "automanip/dispatch/capability/execute_capability.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "automanip/manipulation_server.hpp"

namespace automanip {
namespace dispatch {

bool ExecuteCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

ErrorCode ExecuteCapability::Execute(
    const automsgs::msgs::trajectory_msgs::JointTrajectory& trajectory) {
  return server_->ExecuteTrajectory(trajectory);
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(ExecuteCapability, Capability);

}  // namespace dispatch
}  // namespace automanip
