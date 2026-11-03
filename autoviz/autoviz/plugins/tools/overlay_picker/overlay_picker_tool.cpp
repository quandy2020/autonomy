/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/plugins/tools/overlay_picker/overlay_picker_tool.hpp"

#include <QMouseEvent>

namespace autoviz {
namespace tools {

void OverlayPickerTool::activate(common::ToolContext* context) {
  common::Tool::activate(context);
  last_mouse_.reset();
  left_drag_active_ = false;
  if (context != nullptr && context->set_status) {
    context->set_status(
        QStringLiteral("Overlay picker: drag not yet bound"));
  }
}

bool OverlayPickerTool::mousePressEvent(QMouseEvent* event) {
  if (event == nullptr) {
    return false;
  }
  last_mouse_ = event->pos();
  if (event->button() == Qt::LeftButton) {
    left_drag_active_ = true;
  }
  return false;
}

bool OverlayPickerTool::mouseMoveEvent(QMouseEvent* event) {
  if (event == nullptr) {
    return false;
  }
  last_mouse_ = event->pos();
  if (left_drag_active_ && (event->buttons() & Qt::LeftButton)) {
    // Future: resolve overlay under cursor and apply delta — not wired yet.
  }
  return false;
}

bool OverlayPickerTool::mouseReleaseEvent(QMouseEvent* event) {
  if (event == nullptr) {
    return false;
  }
  last_mouse_ = event->pos();
  if (event->button() == Qt::LeftButton) {
    left_drag_active_ = false;
  }
  return false;
}

QString OverlayPickerTool::statusText() const {
  return QStringLiteral("Overlay picker: drag not yet bound");
}

}  // namespace tools
}  // namespace autoviz
