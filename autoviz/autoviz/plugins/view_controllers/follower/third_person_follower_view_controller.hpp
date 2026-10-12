/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins ThirdPersonFollowerViewController
 * (Willow Garage / Bosch / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file third_person_follower_view_controller.hpp
 * @brief XY-orbit style camera that follows a target frame.
 *
 * @see XYOrbitViewController
 */

#pragma once

#include "autoviz/plugins/view_controllers/xy_orbit/xy_orbit_view_controller.hpp"

namespace autoviz {
namespace display {
namespace view_controllers {

/**
 * @class ThirdPersonFollowerViewController
 * @brief Follows target-frame yaw while keeping XY-orbit interaction.
 */
class ThirdPersonFollowerViewController : public XYOrbitViewController {
 public:
  static constexpr const char* kTypeName = "ThirdPersonFollow";

  explicit ThirdPersonFollowerViewController(
      rendering::ViewController* host = nullptr);

  QString getClassId() const override {
    return QStringLiteral("ThirdPersonFollow");
  }

  static void apply(rendering::ViewController* controller);
};

}  // namespace view_controllers
}  // namespace display
}  // namespace autoviz
