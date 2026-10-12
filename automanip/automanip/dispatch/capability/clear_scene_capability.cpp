/*
 * Copyright 2026 The Openbot Authors
 */

#include "automanip/dispatch/capability/clear_scene_capability.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "automanip/manipulation_server.hpp"

namespace automanip {
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
}  // namespace automanip
