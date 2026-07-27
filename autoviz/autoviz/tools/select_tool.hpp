/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <vector>

#include "autoviz/common/selection.hpp"
#include "autoviz/common/selection_manager.hpp"
#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace tools {

class SelectTool : public common::Tool {
 public:
  std::string id() const override { return "Select"; }
  QString label() const override { return QStringLiteral("Select"); }

  bool mousePressEvent(QMouseEvent* event) override;
  void onDraw(rendering::SceneOverlay& scene) override;
  QString statusText() const override;

  const std::vector<common::SelectionEntry>& selections() const;

 private:
  common::SelectionManager* selectionManager() const;
  void notifySelectionsChanged();
  void drawSelectionMarker(rendering::SceneOverlay& scene,
                           const QVector3D& position) const;

  std::vector<common::SelectionEntry> local_selections_;
};

}  // namespace tools
}  // namespace autoviz
