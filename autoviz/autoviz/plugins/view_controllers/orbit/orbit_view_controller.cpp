/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins OrbitViewController
 * (Willow Garage / Open Source Robotics Foundation / Bosch).
 *****************************************************************************/

#include "autoviz/plugins/view_controllers/orbit/orbit_view_controller.hpp"

namespace autoviz {
namespace display {
namespace view_controllers {

OrbitViewController::OrbitViewController(rendering::ViewController* host)
    : FrameTrackingViewController(host) {}

void OrbitViewController::apply(rendering::ViewController* controller) {
  if (controller != nullptr) {
    controller->setType(rendering::ViewControllerType::kOrbit);
  }
}

void OrbitViewController::yaw(float angle) {
  requireHost();
  host_->orbitBy(angle, 0.f);
}

void OrbitViewController::pitch(float angle) {
  requireHost();
  host_->orbitBy(0.f, angle);
}

void OrbitViewController::zoom(float amount) {
  requireHost();
  // RViz: positive amount moves toward the focal point.
  host_->zoomBy(amount);
}

void OrbitViewController::move(float x, float y, float z) {
  requireHost();
  host_->setTarget(host_->target() + QVector3D(x, y, z));
}

}  // namespace view_controllers
}  // namespace display
}  // namespace autoviz
