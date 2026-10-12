/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins FixedOrientationOrthoViewController
 * (Willow Garage / Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/view_controllers/ortho/fixed_orientation_ortho_view_controller.hpp"

namespace autoviz {
namespace display {
namespace view_controllers {

FixedOrientationOrthoViewController::FixedOrientationOrthoViewController(
    rendering::ViewController* host)
    : FrameTrackingViewController(host) {}

void FixedOrientationOrthoViewController::apply(
    rendering::ViewController* controller) {
  if (controller != nullptr) {
    controller->setType(rendering::ViewControllerType::kTopDownOrtho);
  }
}

void FixedOrientationOrthoViewController::zoom(float amount) {
  requireHost();
  host_->zoomBy(amount);
}

void FixedOrientationOrthoViewController::move(float x, float y, float z) {
  requireHost();
  host_->setTarget(host_->target() + QVector3D(x, y, z));
}

}  // namespace view_controllers
}  // namespace display
}  // namespace autoviz
