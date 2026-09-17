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
 * @brief Capability to save / load world geometry (MoveIt geometry file subset).
 */
class SaveLoadGeometryCapability : public Capability {
 public:
  std::string Name() const override { return "save_load_geometry"; }
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Save current world objects + occupancy to @p path.
   * @param[in] path Output geometry file.
   * @return true on success.
   */
  bool Save(const std::string& path) const;

  /**
   * @brief Load geometry file into the planning scene (clears world first).
   * @param[in] path Input geometry file.
   * @return true on success.
   */
  bool Load(const std::string& path);

 private:
  ManipulationServer* server_ = nullptr;
};

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
