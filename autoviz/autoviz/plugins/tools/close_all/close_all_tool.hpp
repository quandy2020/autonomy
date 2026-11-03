/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

/**
 * @file close_all_tool.hpp
 * @brief Close All displays tool (stub until DisplayGroup API exists).
 */

#pragma once

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace tools {

/**
 * @class CloseAllTool
 * @brief Intended to collapse/disable every display in the tree.
 *
 * Without DisplayGroup access this is a one-shot status message and immediate
 * revert to Move Camera.
 */
class CloseAllTool : public common::Tool {
 public:
  std::string id() const override { return "CloseAll"; }
  QString label() const override { return QStringLiteral("Close All"); }

  void activate(common::ToolContext* context) override;
};

}  // namespace tools
}  // namespace autoviz
