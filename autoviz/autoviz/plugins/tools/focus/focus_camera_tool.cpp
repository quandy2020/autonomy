/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/tools/focus/focus_camera_tool.hpp"

#include <QMouseEvent>

#include "autoviz/plugins/displays/common/screen_projection.hpp"
#include "autoviz/rendering/pick_utils.hpp"
#include "autoviz/rendering/view_controller.hpp"
#include "autoviz/ui/app/icon_loader.hpp"

namespace autoviz {
namespace tools {

void FocusCameraTool::activate(common::ToolContext* context) {
  common::Tool::activate(context);
  hit_cursor_ = IconLoader::toolCursor(QStringLiteral("FocusCamera"));
  std_cursor_ = IconLoader::defaultCursor();
  setCursor(std_cursor_);
}

void FocusCameraTool::deactivate() {
  common::Tool::deactivate();
}

bool FocusCameraTool::resolveLookAtPoint(int x, int y, QVector3D* hit) const {
  if (hit == nullptr || context() == nullptr ||
      context()->view_controller == nullptr) {
    return false;
  }
  const rendering::PickResult pick = rendering::pickAtToolContext(*context(), x, y);
  if (pick.hit) {
    *hit = pick.position;
    return true;
  }
  if (context()->display_context != nullptr &&
      display::unprojectScreenToWorld(context()->display_context,
                                      static_cast<float>(x),
                                      static_cast<float>(y), 1.0f, hit)) {
    return true;
  }
  return context()->view_controller->pickGroundPoint(
      x, y, context()->viewport_width, context()->viewport_height, hit);
}

void FocusCameraTool::updateHover(int x, int y) {
  if (context() == nullptr || context()->view_controller == nullptr) {
    return;
  }
  QVector3D hit;
  const bool success = resolveLookAtPoint(x, y, &hit);
  setCursor(success ? hit_cursor_ : std_cursor_);
  if (context()->set_status) {
    if (success) {
      context()->set_status(
          QStringLiteral("<b>Left-Click:</b> Focus on this point. [%1, %2, %3]")
              .arg(hit.x(), 0, 'f', 3)
              .arg(hit.y(), 0, 'f', 3)
              .arg(hit.z(), 0, 'f', 3));
    } else {
      context()->set_status(statusText());
    }
  }
}

bool FocusCameraTool::mouseMoveEvent(QMouseEvent* event) {
  updateHover(event->pos().x(), event->pos().y());
  return true;
}

bool FocusCameraTool::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() != Qt::LeftButton) {
    return true;
  }
  if (context() == nullptr || context()->view_controller == nullptr) {
    if (context() != nullptr && context()->set_status) {
      context()->set_status(QStringLiteral("Focus Camera: viewport not ready"));
    }
    return true;
  }
  QVector3D hit;
  if (!resolveLookAtPoint(event->pos().x(), event->pos().y(), &hit)) {
    if (context()->set_status) {
      context()->set_status(
          QStringLiteral("Focus Camera: click geometry or the ground plane"));
    }
    return true;
  }
  context()->view_controller->setTarget(hit);
  if (context()->request_redraw) {
    context()->request_redraw();
  }
  if (context()->set_status) {
    context()->set_status(
        QStringLiteral("Focus Camera: target (%1, %2, %3)")
            .arg(hit.x(), 0, 'f', 3)
            .arg(hit.y(), 0, 'f', 3)
            .arg(hit.z(), 0, 'f', 3));
  }
  if (context()->revert_to_default_tool) {
    context()->revert_to_default_tool();
  }
  return true;
}

QString FocusCameraTool::statusText() const {
  return QStringLiteral("<b>Left-Click:</b> Look in this direction.");
}

}  // namespace tools
}  // namespace autoviz
