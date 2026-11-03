/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

/**
 * @file open_all_tool.hpp
 * @brief Open All displays tool (stub until DisplayGroup API exists).
 */

#pragma once

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace tools {

/**
 * @class OpenAllTool
 * @brief Intended to expand/enable every display in the tree.
 *
 * Without DisplayGroup access this is a one-shot status message and immediate
 * revert to Move Camera (RViz @c Tool::Finished pattern).
 */
class OpenAllTool : public common::Tool {
 public:
  std::string id() const override { return "OpenAll"; }
  QString label() const override { return QStringLiteral("Open All"); }

  void activate(common::ToolContext* context) override;
};

}  // namespace tools
}  // namespace autoviz
