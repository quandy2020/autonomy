/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Inspired by RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/plugins/view_controllers/tablet/tablet_view_controller.hpp"

namespace autoviz {
namespace display {
namespace view_controllers {

TabletViewController::TabletViewController(rendering::ViewController* host)
    : FrameTrackingViewController(host) {}

void TabletViewController::apply(rendering::ViewController* controller) {
  if (controller == nullptr) {
    return;
  }
  rendering::ViewState state = controller->state();
  state.type = rendering::ViewControllerType::kTopDownOrtho;
  state.yaw = 0.f;
  state.pitch = 1.45f;
  state.distance = 12.f;
  controller->setState(state);
  controller->setType(rendering::ViewControllerType::kTopDownOrtho);
}

void TabletViewController::zoom(float amount) {
  requireHost();
  host_->zoomBy(amount);
}

void TabletViewController::move(float x, float y, float z) {
  requireHost();
  host_->setTarget(host_->target() + QVector3D(x, y, z));
}

}  // namespace view_controllers
}  // namespace display
}  // namespace autoviz
