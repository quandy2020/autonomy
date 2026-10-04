/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file select_tool.hpp
 * @brief RViz-style Select tool — pick scene objects and draw selection markers.
 *
 * Left-click uses the active viewport's pick path (@c ToolContext::gpu_pick_id_read
 * / @ref common::PickRegistry / @ref common::HandlerManager) to resolve a
 * @ref common::SelectionEntry. Selections are stored both locally and in the
 * shared @ref common::SelectionManager when available, then propagated via
 * @c ToolContext::selections_changed.
 *
 * @see common::Tool
 * @see common::SelectionManager
 * @see InteractTool
 */

#pragma once

#include <vector>

#include "autoviz/common/selection.hpp"
#include "autoviz/common/selection_manager.hpp"
#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace tools {

/**
 * @class SelectTool
 * @brief Interactive tool that picks 3D objects and visualizes the current
 *        selection set.
 *
 * ## Interaction
 *
 * - **Press:** resolve pick at cursor → update @c local_selections_ and notify.
 * - **Draw:** overlay a marker at each selected world position.
 *
 * @note Does not consume camera-drag gestures beyond the pick press; Move
 *       Camera remains the default navigation tool.
 *
 * @see common::SelectionEntry
 * @see common::ToolContext
 */
class SelectTool : public common::Tool {
 public:
  /**
   * @brief Stable tool id used by the tool registry / session config.
   * @return @c "Select".
   */
  std::string id() const override { return "Select"; }

  /**
   * @brief Human-readable toolbar label.
   * @return Localized @c "Select".
   */
  QString label() const override { return QStringLiteral("Select"); }

  /**
   * @brief Handles left-click picking and updates the selection set.
   *
   * @param event Qt mouse press (screen coordinates in the viewport).
   * @return @c true if the pick was handled (consumes default camera press).
   */
  bool mousePressEvent(QMouseEvent* event) override;

  /**
   * @brief Draws selection markers into the active scene overlay.
   *
   * @param scene Overlay canvas owned by the active 3D panel.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

  /**
   * @brief Status-bar text summarizing the current selection count / target.
   * @return Short status string for the frame chrome.
   */
  QString statusText() const override;

  /**
   * @brief Returns the tool-local selection list (last pick results).
   *
   * Prefer @ref common::SelectionManager when the ToolContext provides one;
   * this accessor exposes the tool's own cache for callers that bind directly.
   *
   * @return Const reference to @c local_selections_.
   */
  const std::vector<common::SelectionEntry>& selections() const;

 private:
  /**
   * @brief Resolves @c ToolContext::selection_manager when context is active.
   * @return Non-owning pointer, or @c nullptr if unset.
   */
  common::SelectionManager* selectionManager() const;

  /**
   * @brief Invokes @c ToolContext::selections_changed with the current set.
   */
  void notifySelectionsChanged();

  /**
   * @brief Draws a single world-space selection marker.
   *
   * @param scene Overlay to draw into.
   * @param position World position of the selected entity.
   */
  void drawSelectionMarker(rendering::SceneOverlay& scene,
                           const QVector3D& position) const;

  /** Local copy of the last pick results for this tool instance. */
  std::vector<common::SelectionEntry> local_selections_;
};

}  // namespace tools
}  // namespace autoviz
