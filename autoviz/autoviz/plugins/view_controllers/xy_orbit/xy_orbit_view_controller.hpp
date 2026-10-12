/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins XYOrbitViewController
 * (Willow Garage / Bosch / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file xy_orbit_view_controller.hpp
 * @brief Orbit with constrained pitch (prefer XY plane motion).
 *
 * @see OrbitViewController
 */

#pragma once

#include "autoviz/plugins/view_controllers/orbit/orbit_view_controller.hpp"

namespace autoviz {
namespace display {
namespace view_controllers {

/**
 * @class XYOrbitViewController
 * @brief Orbit variant that keeps pitch fixed (RViz XYOrbit).
 */
class XYOrbitViewController : public OrbitViewController {
 public:
  static constexpr const char* kTypeName = "XYOrbit";

  explicit XYOrbitViewController(rendering::ViewController* host = nullptr);

  QString getClassId() const override { return QStringLiteral("XYOrbit"); }

  static void apply(rendering::ViewController* controller);
};

}  // namespace view_controllers
}  // namespace display
}  // namespace autoviz
