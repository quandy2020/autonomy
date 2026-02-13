/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef AVIZ_COMMON__VIEWPORT_MOUSE_EVENT_HPP_
#define AVIZ_COMMON__VIEWPORT_MOUSE_EVENT_HPP_

#include <QMouseEvent>  // NOLINT: cpplint is unable to handle the include order here
#include <QVector3D>    // NOLINT: cpplint is unable to handle the include order here
#include <QWheelEvent>  // NOLINT: cpplint is unable to handle the include order here

class SceneViewer;

namespace aviz {
namespace common {

/// Wrapper around QMouseEvent and QWheelEvent with viewport-specific information.
class ViewportMouseEvent
{
public:
    ViewportMouseEvent();
    ViewportMouseEvent(QMouseEvent* event, SceneViewer* viewer);
    ViewportMouseEvent(QWheelEvent* event, SceneViewer* viewer);

    /// The x coordinate of the mouse event, in viewport coordinates.
    int x;
    /// The y coordinate of the mouse event, in viewport coordinates.
    int y;

    /// The type of the mouse event (QEvent::MouseButtonPress, etc.).
    QEvent::Type type;

    /// The mouse buttons that were pressed during this event.
    Qt::MouseButtons buttons;

    /// The keyboard modifiers that were active during this event.
    Qt::KeyboardModifiers modifiers;

    /// The amount the wheel was rotated (for wheel events).
    int wheel_delta;

    /// The orientation of the wheel (for wheel events).
    Qt::Orientation wheel_orientation;

    /// The SceneViewer this event occurred in.
    SceneViewer* viewer;

    /// The 3D point in space corresponding to the mouse position (if available).
    QVector3D point3d;

    /// Whether the 3D point is valid.
    bool point3d_valid;
};

}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__VIEWPORT_MOUSE_EVENT_HPP_
