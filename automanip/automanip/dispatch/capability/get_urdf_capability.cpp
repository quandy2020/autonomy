/*
 * Copyright 2026 The Openbot Authors
 */

#include "automanip/dispatch/capability/get_urdf_capability.hpp"

#include <fstream>
#include <string>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "automanip/manipulation_server.hpp"

namespace automanip {
namespace dispatch {

bool GetUrdfCapability::Init(ManipulationServer* server) {
  server_ = server;
  return server_ != nullptr;
}

std::string GetUrdfCapability::UrdfPath() const {
  return server_ ? server_->UrdfPath() : std::string();
}

std::string GetUrdfCapability::UrdfXml() const {
  const std::string path = UrdfPath();
  if (path.empty()) {
    return {};
  }
  std::ifstream in(path);
  if (!in) {
    return {};
  }
  return std::string((std::istreambuf_iterator<char>(in)),
                     std::istreambuf_iterator<char>());
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(GetUrdfCapability, Capability);

}  // namespace dispatch
}  // namespace automanip
