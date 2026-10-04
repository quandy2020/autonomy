/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file title_tools.hpp
 * @brief Foxglove-style panel title-bar tool factories (split / change / expand).
 *
 * Builds compact tool-button strips installed via
 * @ref PanelDockWidget::setTitleBarTools(). Aligns with 3D View title tools:
 * optional reset / settings, Split right | Split down | Change | Expand.
 * Close stays on @ref PanelDockWidget.
 *
 * @see PanelDockWidget
 * @see PanelContextMenuCallbacks
 * @see CreatePanelContextMenu()
 * @see FrameChrome::installStandardPanelTitleTools()
 */

#pragma once

#include <functional>

#include <QString>

class QFrame;
class QIcon;
class QMenu;
class QToolButton;
class QWidget;

namespace autoviz {

struct PanelContextMenuCallbacks;

/**
 * @brief Foxglove-style compact panel title bar stylesheet.
 * @return QSS for standard title tools.
 */
QString PanelTitleToolsStyleSheet();

/**
 * @brief Colorful Plot panel title bar with clearer hover/checked affordance.
 * @return QSS for Plot-specific title tools.
 */
QString PlotTitleToolsStyleSheet();

/**
 * @brief Creates a standard title-bar tool button.
 *
 * @param parent Parent tools host.
 * @param icon Button icon.
 * @param tip Tooltip text.
 * @param checkable Whether the button is checkable.
 * @return New @c QToolButton.
 */
QToolButton* CreateTitleToolButton(QWidget* parent, const QIcon& icon,
                                   const QString& tip, bool checkable = false);

/**
 * @brief Creates a Plot-styled title-bar tool button.
 *
 * @param parent Parent tools host.
 * @param icon Button icon.
 * @param tip Tooltip text.
 * @param checkable Whether the button is checkable.
 * @return New @c QToolButton.
 */
QToolButton* CreatePlotTitleToolButton(QWidget* parent, const QIcon& icon,
                                       const QString& tip,
                                       bool checkable = false);

/**
 * @brief Instant-popup tool button without Qt menu dropdown arrow.
 *
 * Configures @p button to show @p menu on click (More / overflow).
 *
 * @param button Tool button to configure.
 * @param menu Menu to popup (non-owning; typically parented to @p button).
 */
void ConfigurePanelMoreToolButton(QToolButton* button, QMenu* menu);

/**
 * @brief Creates a thin vertical separator for title-bar tool groups.
 * @param parent Parent tools host.
 * @return New separator @c QFrame.
 */
QFrame* CreateTitleSeparator(QWidget* parent);

/**
 * @struct PanelTitleBarOptions
 * @brief Feature flags and callbacks for @ref CreatePanelTitleBarTools().
 *
 * Panel title tools aligned with 3D View:
 * [optional reset] [optional settings] Split right | Split down | Change |
 * Expand. Close stays on @ref PanelDockWidget.
 */
struct PanelTitleBarOptions {
  bool show_reset = false;                 /**< Show reset tool button. */
  std::function<void()> on_reset;          /**< Reset callback. */

  bool show_settings = false;              /**< Show settings toggle. */
  bool settings_checked = false;           /**< Initial settings checked state. */
  std::function<void(bool)> on_settings_toggled; /**< Settings toggled callback. */

  bool show_split = true;                  /**< Show split right / down. */
  bool show_change = true;                 /**< Show change-panel control. */

  bool show_expand = true;                 /**< Show expand / maximize. */
  bool expand_checkable = true;            /**< Expand button is checkable. */
  std::function<void()> on_expand;         /**< Expand callback. */
};

/**
 * @struct PanelTitleBarTools
 * @brief Result of @ref CreatePanelTitleBarTools() — host plus key buttons.
 */
struct PanelTitleBarTools {
  QWidget* widget = nullptr;                 /**< Tools host to install on the dock. */
  QToolButton* settings_button = nullptr;    /**< Settings toggle (may be null). */
  QToolButton* expand_button = nullptr;      /**< Expand toggle (may be null). */
};

/**
 * @brief Builds a full title-bar tools strip from @p callbacks and @p options.
 *
 * @param parent Parent for the tools host (typically the dock).
 * @param callbacks Split / change / expand / remove closures.
 * @param options Feature flags and optional reset/settings hooks.
 * @return Host widget and button pointers.
 *
 * @see CreateStandardPanelTitleTools()
 * @see PanelDockWidget::setTitleBarTools()
 */
PanelTitleBarTools CreatePanelTitleBarTools(
    QWidget* parent, const PanelContextMenuCallbacks& callbacks,
    const PanelTitleBarOptions& options = {});

/**
 * @brief Expand + split/change title bar tools (standard sidebar / main set).
 *
 * Convenience wrapper around @ref CreatePanelTitleBarTools() with default
 * options.
 *
 * @param parent Parent for the tools host.
 * @param callbacks Split / change / expand / remove closures.
 * @return Tools host widget (settings/expand buttons not returned).
 */
QWidget* CreateStandardPanelTitleTools(
    QWidget* parent, const PanelContextMenuCallbacks& callbacks);

}  // namespace autoviz
