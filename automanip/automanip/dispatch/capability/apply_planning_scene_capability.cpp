/*
 * Copyright 2026 The Openbot Authors
 */

#include "automanip/dispatch/capability/apply_planning_scene_capability.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "automanip/manipulation_server.hpp"

namespace automanip {
namespace dispatch {

bool ApplyPlanningSceneCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

bool ApplyPlanningSceneCapability::Apply(const scene::SceneDiff& diff) {
  return server_ && server_->scene_monitor() &&
         server_->scene_monitor()->ApplySceneDiff(diff);
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(ApplyPlanningSceneCapability,
                                        Capability);

}  // namespace dispatch
}  // namespace automanip
