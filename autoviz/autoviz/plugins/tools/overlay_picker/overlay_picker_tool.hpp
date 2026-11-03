/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

/**
 * @file overlay_picker_tool.hpp
 * @brief overlay picker — drag-to-move overlay widgets (stub interaction).
 */

#pragma once

#include <optional>

#include <QPoint>

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace tools {

/**
 * @class OverlayPickerTool
 * @brief Tracks mouse drag for future overlay repositioning.
 *
 * Overlay hit-testing and DisplayGroup wiring are not connected yet; drag
 * events are recorded but do not move overlays.
 */
class OverlayPickerTool : public common::Tool {
 public:
  std::string id() const override { return "OverlayPicker"; }
  QString label() const override {
    return QStringLiteral("Overlay Picker");
  }

  void activate(common::ToolContext* context) override;

  bool mousePressEvent(QMouseEvent* event) override;
  bool mouseMoveEvent(QMouseEvent* event) override;
  bool mouseReleaseEvent(QMouseEvent* event) override;

  QString statusText() const override;

 private:
  std::optional<QPoint> last_mouse_;
  bool left_drag_active_ = false;
};

}  // namespace tools
}  // namespace autoviz
