/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins ThirdPersonFollowerViewController
 * (Willow Garage / Bosch / Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/view_controllers/follower/third_person_follower_view_controller.hpp"

namespace autoviz {
namespace display {
namespace view_controllers {

ThirdPersonFollowerViewController::ThirdPersonFollowerViewController(
    rendering::ViewController* host)
    : XYOrbitViewController(host) {}

void ThirdPersonFollowerViewController::apply(
    rendering::ViewController* controller) {
  if (controller != nullptr) {
    controller->setType(rendering::ViewControllerType::kThirdPersonFollow);
  }
}

}  // namespace view_controllers
}  // namespace display
}  // namespace autoviz
