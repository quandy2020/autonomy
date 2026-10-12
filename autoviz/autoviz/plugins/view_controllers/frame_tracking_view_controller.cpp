/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_common::FramePositionTrackingViewController
 * (Willow Garage / Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/view_controllers/frame_tracking_view_controller.hpp"

#include <stdexcept>

namespace autoviz {
namespace display {
namespace view_controllers {

FrameTrackingViewController::FrameTrackingViewController(
    rendering::ViewController* host)
    : host_(host) {}

void FrameTrackingViewController::requireHost() const {
  if (host_ == nullptr) {
    throw std::logic_error("ViewController host is null");
  }
}

void FrameTrackingViewController::activate() {
  requireHost();
  host_->setTypeByName(getClassId());
}

void FrameTrackingViewController::reset() {
  requireHost();
  host_->reset();
}

void FrameTrackingViewController::setTargetFrame(const QString& frame) {
  requireHost();
  host_->setTargetFrame(frame);
}

QString FrameTrackingViewController::getTargetFrame() const {
  requireHost();
  return host_->targetFrameDisplay();
}

bool FrameTrackingViewController::handleMouseEvent(
    const rendering::ViewportMouseEvent& event) {
  requireHost();
  return host_->handleMouseEvent(event);
}

bool FrameTrackingViewController::handleKeyEvent(int key, bool pressed) {
  requireHost();
  return host_->handleKeyEvent(key, pressed);
}

void FrameTrackingViewController::tick(float delta_seconds) {
  requireHost();
  host_->tick(delta_seconds);
}

QMatrix4x4 FrameTrackingViewController::viewMatrix() const {
  requireHost();
  return host_->viewMatrix();
}

QMatrix4x4 FrameTrackingViewController::projectionMatrix(
    float aspect_ratio) const {
  requireHost();
  return host_->projectionMatrix(aspect_ratio);
}

}  // namespace view_controllers
}  // namespace display
}  // namespace autoviz
