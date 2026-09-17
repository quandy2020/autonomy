/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/dispatch/capability/save_load_geometry_capability.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene_geometry_file.hpp"
#include "autonomy/manipulation/manipulation_server.hpp"

namespace autonomy {
namespace manipulation {
namespace dispatch {

bool SaveLoadGeometryCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

bool SaveLoadGeometryCapability::Save(const std::string& path) const {
  return server_ && server_->scene() &&
         scene::SaveGeometryToFile(*server_->scene(), path);
}

bool SaveLoadGeometryCapability::Load(const std::string& path) {
  return server_ && server_->scene() &&
         scene::LoadGeometryFromFile(server_->scene(), path);
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(SaveLoadGeometryCapability, Capability);

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
