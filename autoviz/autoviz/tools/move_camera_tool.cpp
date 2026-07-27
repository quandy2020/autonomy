/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/tools/move_camera_tool.hpp"

namespace autoviz {
namespace tools {

QString MoveCameraTool::statusText() const {
  return QStringLiteral(
      "Move Camera: Left-Click rotate · Middle-Click / Shift+Left move X/Y · "
      "Right-Click / Wheel zoom · Shift+Right / Shift+Wheel move Z");
}

}  // namespace tools
}  // namespace autoviz
