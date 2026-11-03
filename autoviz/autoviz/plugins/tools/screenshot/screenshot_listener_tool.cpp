/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/plugins/tools/screenshot/screenshot_listener_tool.hpp"

namespace autoviz {
namespace tools {

std::vector<common::DisplayPropertySpec>
ScreenshotListenerTool::propertySpecs() const {
  return {{"screenshot_path", "Screenshot path", "/tmp/autoviz_screenshot.png",
           {}, common::DisplayPropertyKind::kPath}};
}

void ScreenshotListenerTool::activate(common::ToolContext* context) {
  common::Tool::activate(context);
  noted_path_ = propertyValue("screenshot_path", "/tmp/autoviz_screenshot.png");

  QString status =
      QStringLiteral("Screenshot path: %1")
          .arg(QString::fromStdString(noted_path_));
  if (context != nullptr && context->scene_overlay != nullptr) {
    status += QStringLiteral(" (scene overlay ready)");
  } else if (context != nullptr) {
    status += QStringLiteral(" (no scene overlay)");
  }

  if (context != nullptr && context->set_status) {
    context->set_status(status);
  }
}

QString ScreenshotListenerTool::statusText() const {
  if (noted_path_.empty()) {
    return QStringLiteral("Screenshot Listener");
  }
  return QStringLiteral("Screenshot path: %1")
      .arg(QString::fromStdString(noted_path_));
}

}  // namespace tools
}  // namespace autoviz
