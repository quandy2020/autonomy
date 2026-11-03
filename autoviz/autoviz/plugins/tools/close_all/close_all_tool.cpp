/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/plugins/tools/close_all/close_all_tool.hpp"

namespace autoviz {
namespace tools {

void CloseAllTool::activate(common::ToolContext* context) {
  common::Tool::activate(context);
  // TODO: Walk DisplayGroup and collapse/disable all displays.
  if (context != nullptr && context->set_status) {
    context->set_status(
        QStringLiteral("Close All: display group API not wired yet"));
  }
  if (context != nullptr && context->revert_to_default_tool) {
    context->revert_to_default_tool();
  }
}

}  // namespace tools
}  // namespace autoviz
