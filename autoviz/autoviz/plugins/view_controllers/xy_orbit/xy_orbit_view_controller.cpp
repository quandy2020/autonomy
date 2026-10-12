/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins XYOrbitViewController
 * (Willow Garage / Bosch / Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/view_controllers/xy_orbit/xy_orbit_view_controller.hpp"

namespace autoviz {
namespace display {
namespace view_controllers {

XYOrbitViewController::XYOrbitViewController(rendering::ViewController* host)
    : OrbitViewController(host) {}

void XYOrbitViewController::apply(rendering::ViewController* controller) {
  if (controller != nullptr) {
    controller->setType(rendering::ViewControllerType::kXyOrbit);
  }
}

}  // namespace view_controllers
}  // namespace display
}  // namespace autoviz
