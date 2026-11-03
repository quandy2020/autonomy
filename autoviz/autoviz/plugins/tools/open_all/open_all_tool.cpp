/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/plugins/tools/open_all/open_all_tool.hpp"

namespace autoviz {
namespace tools {

void OpenAllTool::activate(common::ToolContext* context) {
  common::Tool::activate(context);
  // TODO: Walk DisplayGroup and set enabled/expanded on all displays.
  if (context != nullptr && context->set_status) {
    context->set_status(
        QStringLiteral("Open All: display group API not wired yet"));
  }
  if (context != nullptr && context->revert_to_default_tool) {
    context->revert_to_default_tool();
  }
}

}  // namespace tools
}  // namespace autoviz
