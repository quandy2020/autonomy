/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file selection_panel.hpp
 * @brief Sidebar list of the current 3D / tool selection entries.
 *
 * Mirrors @ref common::SelectionEntry objects from VisualizationManager so the
 * user can see what the Interact / Select tools have picked.
 *
 * @see PropertyInspectorPanel
 * @see common::SelectionEntry
 * @see common::VisualizationManager
 */

#pragma once

#include <vector>

#include <QListWidget>
#include <QWidget>

class QPaintEvent;

#include "autoviz/common/selection.hpp"

namespace autoviz {

namespace common {
class VisualizationManager;
}

/**
 * @class SelectionPanel
 * @brief Read-mostly list panel for active selection entries.
 *
 * ## Layout
 *
 * @code
 * ┌─────────────────────────────┐
 * │  entry 0                    │
 * │  entry 1                    │
 * │  …                          │
 * └─────────────────────────────┘
 * @endcode
 *
 * Callers push updates via @ref setSelections() whenever the manager's
 * selection set changes. The panel paints frosted chrome in @ref paintEvent().
 *
 * @note Non-owning @c manager_ is reserved for future jump-to / highlight
 *       actions; the list itself is driven solely by @ref setSelections().
 */
class SelectionPanel : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs an empty selection list.
   *
   * @param manager Non-owning visualization manager (may be @c nullptr).
   * @param parent Qt parent widget.
   */
  explicit SelectionPanel(common::VisualizationManager* manager,
                          QWidget* parent = nullptr);

  /**
   * @brief Replaces the list contents with @p entries.
   *
   * Clears @c list_ and appends one row per entry (display string derived from
   * the SelectionEntry fields). Also caches @p entries in @c entries_.
   *
   * @param entries Current selection set from the manager / tools.
   */
  void setSelections(const std::vector<common::SelectionEntry>& entries);

 protected:
  /**
   * @brief Paints the frosted glass panel chrome behind the list.
   *
   * @param event Paint event (rectangle unused; paints @c rect()).
   */
  void paintEvent(QPaintEvent* event) override;

 private:
  /** Non-owning; available for selection-related manager queries. */
  common::VisualizationManager* manager_ = nullptr;

  /** List widget showing one row per selection entry. */
  QListWidget* list_ = nullptr;

  /** Cached copy of the last @ref setSelections() argument. */
  std::vector<common::SelectionEntry> entries_;
};

}  // namespace autoviz
