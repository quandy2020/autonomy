/*
 * Copyright 2026 The Openbot Authors
 */

#include "automanip/dispatch/capability/clear_octomap_capability.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "automanip/manipulation_server.hpp"

namespace automanip {
namespace dispatch {

bool ClearOctomapCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

bool ClearOctomapCapability::Clear() {
  return server_ && server_->scene_monitor() &&
         server_->scene_monitor()->ClearOctomap();
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(ClearOctomapCapability, Capability);

}  // namespace dispatch
}  // namespace automanip
