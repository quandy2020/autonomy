/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins FPSViewController
 * (Willow Garage / Bosch / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file fps_view_controller.hpp
 * @brief First-person camera (yaw / pitch / eye position + WASD).
 *
 * @see FrameTrackingViewController
 */

#pragma once

#include "autoviz/plugins/view_controllers/frame_tracking_view_controller.hpp"

namespace autoviz {
namespace display {
namespace view_controllers {

/**
 * @class FPSViewController
 * @brief FPS look + walk/fly motion (RViz FPS).
 */
class FPSViewController : public FrameTrackingViewController {
 public:
  static constexpr const char* kTypeName = "FPS";

  explicit FPSViewController(rendering::ViewController* host = nullptr);

  QString getClassId() const override { return QStringLiteral("FPS"); }

  static void apply(rendering::ViewController* controller);

  void yaw(float angle);
  void pitch(float angle);
  void move(float x, float y, float z);

  void setFpsKey(int key, bool pressed);
  bool fpsFlyMode() const;
};

}  // namespace view_controllers
}  // namespace display
}  // namespace autoviz
