/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/manipulation/dispatch/capability/capability.hpp"
#include "autonomy/manipulation/motion/scene/scene_monitor.hpp"

namespace autonomy {
namespace manipulation {
namespace dispatch {

/**
 * @brief Capability that applies a SceneDiff to the planning scene.
 */
class ApplyPlanningSceneCapability : public Capability {
 public:
  /** @brief Returns "apply_planning_scene". */
  std::string Name() const override { return "apply_planning_scene"; }

  /**
   * @brief Bind to @p server.
   * @param[in] server Runtime server (not owned).
   * @return true if @p server is non-null.
   */
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Apply @p diff to the server's planning scene.
   * @param[in] diff Incremental scene update.
   * @return true if the diff was applied.
   */
  bool Apply(const scene::SceneDiff& diff);

 private:
  ManipulationServer* server_ = nullptr;
};

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
