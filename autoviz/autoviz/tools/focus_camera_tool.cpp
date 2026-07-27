/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/tools/focus_camera_tool.hpp"

#include <QMouseEvent>

#include "autoviz/rendering/pick_utils.hpp"
#include "autoviz/rendering/view_controller.hpp"

namespace autoviz {
namespace tools {

bool FocusCameraTool::mousePressEvent(QMouseEvent* event) {
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
  const rendering::PickResult pick = rendering::pickAtToolContext(
      *context(), event->pos().x(), event->pos().y());
  if (pick.hit) {
    hit = pick.position;
  } else if (!context()->view_controller->pickGroundPoint(
                 event->pos().x(), event->pos().y(), context()->viewport_width,
                 context()->viewport_height, &hit)) {
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
  return true;
}

QString FocusCameraTool::statusText() const {
  return QStringLiteral("Focus Camera: click to move orbit target");
}

}  // namespace tools
}  // namespace autoviz
