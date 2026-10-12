/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file select_tool.hpp
 * @brief RViz-style Select tool — pick scene objects and draw selection markers.
 */

#pragma once

#include <vector>

#include <Qt>

#include "autoviz/common/selection.hpp"
#include "autoviz/rendering/pick_utils.hpp"
#include "autoviz/common/selection_manager.hpp"
#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace tools {

class SelectTool : public common::Tool {
 public:
  std::string id() const override { return "Select"; }
  QString label() const override { return QStringLiteral("Select"); }
  char shortcutKey() const override { return 's'; }

  bool keyPressEvent(QKeyEvent* event) override;
  bool mousePressEvent(QMouseEvent* event) override;
  bool mouseMoveEvent(QMouseEvent* event) override;
  bool mouseReleaseEvent(QMouseEvent* event) override;
  void onDraw(rendering::SceneOverlay& scene) override;
  QString statusText() const override;

  const std::vector<common::SelectionEntry>& selections() const;

 private:
  common::SelectionManager* selectionManager() const;
  void notifySelectionsChanged();
  void drawSelectionMarker(rendering::SceneOverlay& scene,
                           const QVector3D& position) const;
  void drawSelectionRectangle(rendering::SceneOverlay& scene) const;
  void applyPickResult(const rendering::PickResult& pick,
                       common::SelectionManager::SelectMode mode);
  void selectInRectangle(int x0, int y0, int x1, int y1,
                         common::SelectionManager::SelectMode mode);
  static common::SelectionManager::SelectMode modeFromModifiers(
      Qt::KeyboardModifiers modifiers);

  std::vector<common::SelectionEntry> local_selections_;
  bool selecting_ = false;
  int sel_start_x_ = 0;
  int sel_start_y_ = 0;
  int sel_end_x_ = 0;
  int sel_end_y_ = 0;
};

}  // namespace tools
}  // namespace autoviz
