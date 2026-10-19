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
  if (controller == nullptr) {
    return;
  }
  const auto prev = controller->type();
  controller->setType(rendering::ViewControllerType::kThirdPersonFollow);
  // Entering chase view: mount on base_link behind the robot. Leaving an
  // Orbit/FPS sky camera or a camera_link target otherwise looks "wrong".
  if (prev != rendering::ViewControllerType::kThirdPersonFollow) {
    if (!controller->placeThirdPersonOnRobotFrame()) {
      rendering::ViewState state = controller->state();
      state.target = QVector3D(0.f, 0.f, 0.f);
      state.distance = 2.5f;
      state.yaw = 3.14159265f;
      state.pitch = 0.55f;
      controller->setState(state);
    }
  }
}

}  // namespace view_controllers
}  // namespace display
}  // namespace autoviz
