/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Inspired by RViz visualization plugins (BSD).
 *****************************************************************************/

/**
 * @file tablet_view_controller.hpp
 * @brief Tablet-style top-down orthographic view for teleop panels.
 *
 * @see FixedOrientationOrthoViewController
 */

#pragma once

#include "autoviz/plugins/view_controllers/frame_tracking_view_controller.hpp"

namespace autoviz {
namespace display {
namespace view_controllers {

/**
 * @class TabletViewController
 * @brief Orthographic top-down camera with tablet defaults.
 */
class TabletViewController : public FrameTrackingViewController {
 public:
  static constexpr const char* kTypeName = "Tablet";

  explicit TabletViewController(rendering::ViewController* host = nullptr);

  QString getClassId() const override { return QStringLiteral("Tablet"); }

  static void apply(rendering::ViewController* controller);

  void zoom(float amount);
  void move(float x, float y, float z);
};

}  // namespace view_controllers
}  // namespace display
}  // namespace autoviz
