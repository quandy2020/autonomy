/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "automanip/dispatch/capability/capability.hpp"

namespace automanip {
namespace dispatch {

/**
 * @brief Capability that clears occupancy (MoveIt clear_octomap).
 */
class ClearOctomapCapability : public Capability {
 public:
  std::string Name() const override { return "clear_octomap"; }
  bool Init(ManipulationServer* server) override;
  /** @brief Clear occupied points on the monitored scene. */
  bool Clear();

 private:
  ManipulationServer* server_ = nullptr;
};

}  // namespace dispatch
}  // namespace automanip
