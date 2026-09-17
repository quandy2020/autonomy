/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/dispatch/capability/get_planning_scene_capability.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/manipulation_server.hpp"

namespace autonomy {
namespace manipulation {
namespace dispatch {

bool GetPlanningSceneCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

std::shared_ptr<scene::PlanningScene> GetPlanningSceneCapability::Scene()
    const {
  if (!server_ || !server_->scene_monitor()) {
    return nullptr;
  }
  return server_->scene_monitor()->Scene();
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(GetPlanningSceneCapability, Capability);

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
