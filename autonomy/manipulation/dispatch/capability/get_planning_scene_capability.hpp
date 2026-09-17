/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <memory>

#include "autonomy/manipulation/dispatch/capability/capability.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace dispatch {

/**
 * @brief Capability that exposes the current planning scene.
 */
class GetPlanningSceneCapability : public Capability {
 public:
  /** @brief Returns "get_planning_scene". */
  std::string Name() const override { return "get_planning_scene"; }

  /**
   * @brief Bind to @p server.
   * @param[in] server Runtime server (not owned).
   * @return true if @p server is non-null.
   */
  bool Init(ManipulationServer* server) override;

  /**
   * @brief Shared pointer to the server's planning scene.
   * @return Scene pointer, or empty if unavailable.
   */
  std::shared_ptr<scene::PlanningScene> Scene() const;

 private:
  ManipulationServer* server_ = nullptr;
};

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
