/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QMouseEvent>
#include <QWheelEvent>

#include "autoviz/rendering/view_controller.hpp"

namespace autoviz {
namespace rendering {

inline ViewportMouseEvent MakeViewportPressEvent(const QMouseEvent& event,
                                                int viewport_width,
                                                int viewport_height) {
  ViewportMouseEvent viewport_event;
  viewport_event.action = ViewportMouseEvent::Action::kPress;
  viewport_event.buttons = event.buttons();
  viewport_event.modifiers = event.modifiers();
  viewport_event.x = event.pos().x();
  viewport_event.y = event.pos().y();
  viewport_event.viewport_width = viewport_width;
  viewport_event.viewport_height = viewport_height;
  return viewport_event;
}

inline ViewportMouseEvent MakeViewportMoveEvent(const QMouseEvent& event,
                                                int viewport_width,
                                                int viewport_height) {
  ViewportMouseEvent viewport_event;
  viewport_event.action = ViewportMouseEvent::Action::kMove;
  viewport_event.buttons = event.buttons();
  viewport_event.modifiers = event.modifiers();
  viewport_event.x = event.pos().x();
  viewport_event.y = event.pos().y();
  viewport_event.viewport_width = viewport_width;
  viewport_event.viewport_height = viewport_height;
  return viewport_event;
}

inline ViewportMouseEvent MakeViewportReleaseEvent(const QMouseEvent& event,
                                                   int viewport_width,
                                                   int viewport_height) {
  ViewportMouseEvent viewport_event;
  viewport_event.action = ViewportMouseEvent::Action::kRelease;
  viewport_event.buttons = event.buttons();
  viewport_event.modifiers = event.modifiers();
  viewport_event.x = event.pos().x();
  viewport_event.y = event.pos().y();
  viewport_event.viewport_width = viewport_width;
  viewport_event.viewport_height = viewport_height;
  return viewport_event;
}

inline ViewportMouseEvent MakeViewportWheelEvent(const QWheelEvent& event,
                                                 int viewport_width,
                                                 int viewport_height) {
  ViewportMouseEvent viewport_event;
  viewport_event.action = ViewportMouseEvent::Action::kWheel;
  viewport_event.modifiers = event.modifiers();
  viewport_event.wheel_delta = event.angleDelta().y();
  viewport_event.viewport_width = viewport_width;
  viewport_event.viewport_height = viewport_height;
  return viewport_event;
}

}  // namespace rendering
}  // namespace autoviz
