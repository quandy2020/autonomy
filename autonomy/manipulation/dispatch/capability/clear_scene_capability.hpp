/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include "autonomy/manipulation/dispatch/capability/capability.hpp"

namespace autonomy {
namespace manipulation {
namespace dispatch {

/**
 * @brief Capability that clears world collision objects (optional attachments).
 */
class ClearSceneCapability : public Capability {
 public:
  std::string Name() const override { return "clear_scene"; }
  bool Init(ManipulationServer* server) override;
  /**
   * @brief Clear world objects.
   * @param[in] clear_attached Also clear attachments when true.
   */
  bool Clear(bool clear_attached = false);

 private:
  ManipulationServer* server_ = nullptr;
};

}  // namespace dispatch
}  // namespace manipulation
}  // namespace autonomy
