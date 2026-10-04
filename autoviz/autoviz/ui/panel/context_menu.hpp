/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file context_menu.hpp
 * @brief Foxglove-style panel overflow menu factory (Change / Split / Expand /
 *        Remove).
 *
 * Builds a @c QMenu from @ref PanelContextMenuCallbacks. Shared by title-bar
 * “more” buttons and right-click contexts.
 *
 * @see CreatePanelTitleBarTools()
 * @see FrameChrome::makePanelContextMenuCallbacks()
 * @see ChangePanelMenuWidget
 */

#pragma once

#include <Qt>

#include <functional>

#include <QString>

class QMenu;
class QWidget;

namespace autoviz {

/**
 * @struct PanelContextMenuCallbacks
 * @brief Closures and context for @ref CreatePanelContextMenu().
 *
 * All functions may be empty; missing callbacks omit the corresponding actions.
 */
struct PanelContextMenuCallbacks {
  /** Object name of the dock the menu operates on (Change panel filter). */
  QString current_object_name;

  /** Switch this dock to another panel type. */
  std::function<void(const QString& object_name)> change_panel;

  /** Split right (@c Qt::Horizontal) or down (@c Qt::Vertical). */
  std::function<void(Qt::Orientation orientation)> split;

  /** Expand / maximize this panel in the center host. */
  std::function<void()> expand;

  /** Remove / close this panel. */
  std::function<void()> remove;

  /**
   * When set, adds “Download plot data as CSV” (Plot panel).
   */
  std::function<void()> download_plot_csv;

  /**
   * When set, adds “Download plot as PNG” (Plot panel).
   */
  std::function<void()> download_plot_png;

  /**
   * When set, adds “Download image as PNG” (Image panel).
   */
  std::function<void()> download_image_png;
};

/**
 * @brief Foxglove-style panel overflow menu (Change panel, Split, Expand,
 *        Remove).
 *
 * Ownership of the returned menu is transferred to the caller (typically
 * parented to @p parent or a tool button).
 *
 * @param parent Parent widget for the menu.
 * @param callbacks Action closures; empty functions omit menu entries.
 * @return New @c QMenu instance.
 *
 * @see PanelContextMenuCallbacks
 * @see ConfigurePanelMoreToolButton()
 */
QMenu* CreatePanelContextMenu(QWidget* parent,
                              const PanelContextMenuCallbacks& callbacks);

}  // namespace autoviz
