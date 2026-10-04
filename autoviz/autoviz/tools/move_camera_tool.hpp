/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file move_camera_tool.hpp
 * @brief Default Move Camera tool — viewport navigation via ViewController.
 *
 * RViz @c MoveCameraTool equivalent. Does not override mouse handlers; the
 * frame / viewport routes unconsumed events to @ref rendering::ViewController
 * for orbit / pan / zoom. This tool exists mainly as the default "finished"
 * target after one-shot tools (@c ToolContext::revert_to_default_tool).
 *
 * @see common::Tool
 * @see rendering::ViewController
 * @see FocusCameraTool
 */

#pragma once

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace tools {

/**
 * @class MoveCameraTool
 * @brief Passive default tool whose label/status identify camera-navigation mode.
 *
 * Mouse / wheel handling is intentionally left to the base @ref common::Tool
 * (returns @c false) so the viewport applies ViewController gestures.
 */
class MoveCameraTool : public common::Tool {
 public:
  /**
   * @brief Stable tool id for the registry / session config.
   * @return @c "MoveCamera".
   */
  std::string id() const override { return "MoveCamera"; }

  /**
   * @brief Human-readable toolbar label.
   * @return Localized @c "Move Camera".
   */
  QString label() const override { return QStringLiteral("Move Camera"); }

  /**
   * @brief Brief usage hint for the frame status bar.
   * @return Instructional status string.
   */
  QString statusText() const override;
};

}  // namespace tools
}  // namespace autoviz
