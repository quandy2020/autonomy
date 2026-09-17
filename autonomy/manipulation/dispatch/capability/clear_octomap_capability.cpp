/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/dispatch/capability/clear_octomap_capability.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/manipulation_server.hpp"

namespace autonomy {
namespace manipulation {
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
}  // namespace manipulation
}  // namespace autonomy
