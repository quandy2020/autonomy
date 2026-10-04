/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file viewport_mouse.hpp
 * @brief Convert Qt mouse / wheel events into @ref ViewportMouseEvent.
 *
 * Thin inline helpers used by @ref RenderWindow and @ref OgreRenderWindow so
 * @ref ViewController::handleMouseEvent() stays backend-agnostic.
 *
 * @see ViewportMouseEvent
 * @see ViewController::handleMouseEvent()
 */

#pragma once

#include <QMouseEvent>
#include <QWheelEvent>

#include "autoviz/rendering/view_controller.hpp"

namespace autoviz {
namespace rendering {

/**
 * @brief Builds a press @ref ViewportMouseEvent from a Qt mouse press.
 *
 * @param event Qt mouse event (buttons, modifiers, position).
 * @param viewport_width Current viewport width in widget pixels.
 * @param viewport_height Current viewport height in widget pixels.
 * @return Event with @ref ViewportMouseEvent::Action::kPress.
 */
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

/**
 * @brief Builds a move @ref ViewportMouseEvent from a Qt mouse move.
 *
 * @param event Qt mouse event.
 * @param viewport_width Viewport width.
 * @param viewport_height Viewport height.
 * @return Event with @ref ViewportMouseEvent::Action::kMove.
 */
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

/**
 * @brief Builds a release @ref ViewportMouseEvent from a Qt mouse release.
 *
 * @param event Qt mouse event.
 * @param viewport_width Viewport width.
 * @param viewport_height Viewport height.
 * @return Event with @ref ViewportMouseEvent::Action::kRelease.
 */
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

/**
 * @brief Builds a wheel @ref ViewportMouseEvent from a Qt wheel event.
 *
 * Stores vertical @c angleDelta().y() in @c wheel_delta. Position fields are
 * left at defaults (zoom is typically cursor-independent in Orbit).
 *
 * @param event Qt wheel event.
 * @param viewport_width Viewport width.
 * @param viewport_height Viewport height.
 * @return Event with @ref ViewportMouseEvent::Action::kWheel.
 */
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
