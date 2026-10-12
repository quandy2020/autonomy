/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins FrameViewController
 * (Willow Garage / Bosch / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file frame_view_controller.hpp
 * @brief Frame-aligned FPS camera (RViz FrameViewController).
 *
 * Autoviz maps this to the FPS backend (@c rendering::ViewControllerType::kFps)
 * with target-frame tracking; there is no separate enum value.
 *
 * @see FPSViewController
 */

#pragma once

#include "autoviz/plugins/view_controllers/fps/fps_view_controller.hpp"

namespace autoviz {
namespace display {
namespace view_controllers {

/**
 * @class FrameViewController
 * @brief FPS camera kept aligned with a tracked TF frame.
 */
class FrameViewController : public FPSViewController {
 public:
  /** Session / RViz short name; Views panel still lists @c FPS. */
  static constexpr const char* kTypeName = "Frame";

  explicit FrameViewController(rendering::ViewController* host = nullptr);

  QString getClassId() const override { return QStringLiteral("FPS"); }

  static void apply(rendering::ViewController* controller);
};

}  // namespace view_controllers
}  // namespace display
}  // namespace autoviz
