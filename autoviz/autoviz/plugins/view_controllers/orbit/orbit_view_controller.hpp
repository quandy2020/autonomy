/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins OrbitViewController
 * (Willow Garage / Open Source Robotics Foundation / Bosch).
 *****************************************************************************/

/**
 * @file orbit_view_controller.hpp
 * @brief Orbital camera about a focal point (RViz Orbit).
 *
 * @see FrameTrackingViewController
 * @see rendering::ViewController
 */

#pragma once

#include "autoviz/plugins/view_controllers/frame_tracking_view_controller.hpp"

namespace autoviz {
namespace display {
namespace view_controllers {

/**
 * @class OrbitViewController
 * @brief Yaw / pitch / distance orbit around a focal point.
 */
class OrbitViewController : public FrameTrackingViewController {
 public:
  static constexpr const char* kTypeName = "Orbit";

  explicit OrbitViewController(rendering::ViewController* host = nullptr);

  QString getClassId() const override { return QStringLiteral("Orbit"); }

  /** Applies Orbit mode to @p controller (registry entry point). */
  static void apply(rendering::ViewController* controller);

  void yaw(float angle);
  void pitch(float angle);
  void zoom(float amount);
  void move(float x, float y, float z);
};

}  // namespace view_controllers
}  // namespace display
}  // namespace autoviz
