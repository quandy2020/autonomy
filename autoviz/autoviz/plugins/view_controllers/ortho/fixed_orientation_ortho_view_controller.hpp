/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins FixedOrientationOrthoViewController
 * (Willow Garage / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file fixed_orientation_ortho_view_controller.hpp
 * @brief Orthographic top-down view (Autoviz type id @c TopDownOrtho).
 *
 * RViz class name is FixedOrientationOrthoViewController; the Views panel and
 * session config use @c TopDownOrtho.
 *
 * @see FrameTrackingViewController
 */

#pragma once

#include "autoviz/plugins/view_controllers/frame_tracking_view_controller.hpp"

namespace autoviz {
namespace display {
namespace view_controllers {

/**
 * @class FixedOrientationOrthoViewController
 * @brief Fixed-orientation orthographic camera (RViz TopDownOrtho).
 */
class FixedOrientationOrthoViewController : public FrameTrackingViewController {
 public:
  static constexpr const char* kTypeName = "TopDownOrtho";

  explicit FixedOrientationOrthoViewController(
      rendering::ViewController* host = nullptr);

  QString getClassId() const override { return QStringLiteral("TopDownOrtho"); }

  static void apply(rendering::ViewController* controller);

  void zoom(float amount);
  void move(float x, float y, float z);
};

}  // namespace view_controllers
}  // namespace display
}  // namespace autoviz
