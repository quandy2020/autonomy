/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <string>

#include "autonomy/manipulation/dispatch/capability/capability.hpp"

namespace autonomy {
namespace manipulation {
namespace dispatch {

/**
 * @brief Capability returning the loaded robot URDF path / text (get_urdf).
 */
class GetUrdfCapability : public Capability {
 public:
  std::string Name() const override { return "get_urdf"; }
  bool Init(ManipulationServer* server) override;

  /** @brief Resolved URDF file path used at Init. */
  std::string UrdfPath() const;

  /**
   * @brief Read URDF file contents (empty if unreadable).
   * @return URDF XML text.
   */
  std::string UrdfXml() const;

 private:
  ManipulationServer* server_ = nullptr;
};

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
