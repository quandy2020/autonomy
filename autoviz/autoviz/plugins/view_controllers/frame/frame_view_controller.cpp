/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins FrameViewController
 * (Willow Garage / Bosch / Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/view_controllers/frame/frame_view_controller.hpp"

namespace autoviz {
namespace display {
namespace view_controllers {

FrameViewController::FrameViewController(rendering::ViewController* host)
    : FPSViewController(host) {}

void FrameViewController::apply(rendering::ViewController* controller) {
  // RViz FrameViewController subclasses FPS; Autoviz uses the same backend.
  FPSViewController::apply(controller);
}

}  // namespace view_controllers
}  // namespace display
}  // namespace autoviz
