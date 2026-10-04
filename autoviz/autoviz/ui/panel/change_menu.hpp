/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file change_menu.hpp
 * @brief Searchable panel picker widget (Foxglove Change panel submenu).
 *
 * Embedded in panel context / overflow menus. Emits @ref panelSelected() with
 * the chosen dock object name when the user activates a list row.
 *
 * @see CreatePanelContextMenu()
 * @see PanelCatalog()
 * @see FrameLayout::changePanelInDock()
 */

#pragma once

#include <QWidget>

class QLineEdit;
class QListWidget;

namespace autoviz {

/**
 * @class ChangePanelMenuWidget
 * @brief Searchable panel picker (Foxglove Change panel submenu).
 *
 * ## Layout
 *
 * @code
 * ┌─────────────────────────┐
 * │ [Search…            ]   │
 * ├─────────────────────────┤
 * │  🖼 Image               │
 * │  📈 Plot                │
 * │  …                      │
 * └─────────────────────────┘
 * @endcode
 *
 * Filter is case-insensitive on label / object name. @c selecting_ guards
 * re-entrancy while emitting @ref panelSelected().
 *
 * @see PanelCatalog()
 * @see CreatePanelContextMenu()
 */
class ChangePanelMenuWidget : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Builds search field + list and populates from @ref PanelCatalog().
   * @param parent Typically a @c QWidgetAction host inside a @c QMenu.
   */
  explicit ChangePanelMenuWidget(QWidget* parent = nullptr);

 signals:
  /**
   * @brief Emitted when the user activates a catalog row.
   * @param object_name Dock object name / type id to switch to.
   */
  void panelSelected(const QString& object_name);

 private slots:
  /**
   * @brief Filters the list as the search text changes.
   * @param text Current filter string.
   */
  void onFilterChanged(const QString& text);

  /**
   * @brief Emits @ref panelSelected() for the current list item.
   */
  void onItemActivated();

 private:
  /**
   * @brief Fills @c list_ from @ref PanelCatalog() (implemented entries).
   */
  void populate();

  /** Filter line edit. */
  QLineEdit* search_ = nullptr;

  /** Selectable panel list. */
  QListWidget* list_ = nullptr;

  /** Re-entrancy guard while emitting selection. */
  bool selecting_ = false;
};

}  // namespace autoviz
