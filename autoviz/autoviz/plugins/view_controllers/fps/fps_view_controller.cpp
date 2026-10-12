/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins FPSViewController
 * (Willow Garage / Bosch / Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/view_controllers/fps/fps_view_controller.hpp"

namespace autoviz {
namespace display {
namespace view_controllers {

FPSViewController::FPSViewController(rendering::ViewController* host)
    : FrameTrackingViewController(host) {}

void FPSViewController::apply(rendering::ViewController* controller) {
  if (controller != nullptr) {
    controller->setType(rendering::ViewControllerType::kFps);
  }
}

void FPSViewController::yaw(float angle) {
  requireHost();
  host_->orbitBy(angle, 0.f);
}

void FPSViewController::pitch(float angle) {
  requireHost();
  host_->orbitBy(0.f, angle);
}

void FPSViewController::move(float x, float y, float z) {
  requireHost();
  rendering::ViewState state = host_->state();
  state.fps_position += QVector3D(x, y, z);
  host_->setState(state);
}

void FPSViewController::setFpsKey(int key, bool pressed) {
  requireHost();
  host_->setFpsKey(key, pressed);
}

bool FPSViewController::fpsFlyMode() const {
  requireHost();
  return host_->fpsFlyMode();
}

}  // namespace view_controllers
}  // namespace display
}  // namespace autoviz
