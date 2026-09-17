/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/dispatch/capability/clear_scene_capability.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/manipulation_server.hpp"

namespace autonomy {
namespace manipulation {
namespace dispatch {

bool ClearSceneCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

bool ClearSceneCapability::Clear(bool clear_attached) {
  return server_ && server_->scene_monitor() &&
         server_->scene_monitor()->ClearScene(clear_attached);
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(ClearSceneCapability, Capability);

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
