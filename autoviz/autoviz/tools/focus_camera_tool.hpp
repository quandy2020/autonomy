/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file focus_camera_tool.hpp
 * @brief Focus Camera tool — click to set the view focal / look-at point.
 *
 * RViz @c FocusTool equivalent. Left-click ray-picks a world point and moves
 * the active @ref rendering::ViewController focus (orbit target / FPS look)
 * to that location.
 *
 * @see MoveCameraTool
 * @see rendering::ViewController
 * @see common::Tool
 */

#pragma once

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace tools {

/**
 * @class FocusCameraTool
 * @brief One-shot click tool that recenters the camera on a picked point.
 *
 * After a successful focus, the frame typically reverts to
 * @ref MoveCameraTool via @c ToolContext::revert_to_default_tool.
 */
class FocusCameraTool : public common::Tool {
 public:
  /**
   * @brief Stable tool id for the registry / session config.
   * @return @c "FocusCamera".
   */
  std::string id() const override { return "FocusCamera"; }

  /**
   * @brief Human-readable toolbar label.
   * @return Localized @c "Focus Camera".
   */
  QString label() const override { return QStringLiteral("Focus Camera"); }

  /**
   * @brief Picks a world point and updates the view controller focus.
   *
   * @param event Mouse press in viewport coordinates.
   * @return @c true if a pick succeeded and focus was updated.
   */
  bool mousePressEvent(QMouseEvent* event) override;

  /**
   * @brief Brief usage hint for the frame status bar.
   * @return Instructional status string.
   */
  QString statusText() const override;
};

}  // namespace tools
}  // namespace autoviz
