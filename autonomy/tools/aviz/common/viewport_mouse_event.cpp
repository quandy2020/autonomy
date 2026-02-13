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

#include "autonomy/tools/aviz/common/viewport_mouse_event.hpp"

#include <QMouseEvent>
#include <QWheelEvent>

namespace aviz {
namespace common {

ViewportMouseEvent::ViewportMouseEvent()
    : x(0),
      y(0),
      type(QEvent::None),
      buttons(Qt::NoButton),
      modifiers(Qt::NoModifier),
      wheel_delta(0),
      wheel_orientation(Qt::Vertical),
      viewer(nullptr),
      point3d_valid(false) {}

ViewportMouseEvent::ViewportMouseEvent(QMouseEvent* event, SceneViewer* viewer)
    : x(event->x()),
      y(event->y()),
      type(event->type()),
      buttons(event->buttons()),
      modifiers(event->modifiers()),
      wheel_delta(0),
      wheel_orientation(Qt::Vertical),
      viewer(viewer),
      point3d_valid(false) {}

ViewportMouseEvent::ViewportMouseEvent(QWheelEvent* event, SceneViewer* viewer)
    : x(event->x()),
      y(event->y()),
      type(event->type()),
      buttons(event->buttons()),
      modifiers(event->modifiers()),
      wheel_delta(event->angleDelta().y()),
      wheel_orientation(event->orientation()),
      viewer(viewer),
      point3d_valid(false) {}

}  // namespace common
}  // namespace aviz
