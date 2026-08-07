/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

/** Foxglove-style compact panel title bar styling. */
QString PanelTitleToolsStyleSheet();

/** Colorful Plot panel title bar with clearer hover/checked affordance. */
QString PlotTitleToolsStyleSheet();

QToolButton* CreateTitleToolButton(QWidget* parent, const QIcon& icon,
                                   const QString& tip, bool checkable = false);

QToolButton* CreatePlotTitleToolButton(QWidget* parent, const QIcon& icon,
                                       const QString& tip, bool checkable = false);

/** Foxglove-style More button: icon only, no Qt menu dropdown arrow. */
void ConfigurePanelMoreToolButton(QToolButton* button, QMenu* menu);

QFrame* CreateTitleSeparator(QWidget* parent);

/** RViz2-style panel title bar: optional reset/settings, expand, overflow menu. */
struct PanelTitleBarOptions {
  bool show_reset = false;
  std::function<void()> on_reset;

  bool show_settings = false;
  bool settings_checked = false;
  std::function<void(bool)> on_settings_toggled;

  bool show_expand = true;
  bool expand_checkable = true;
  std::function<void()> on_expand;

  bool show_more = true;
};

struct PanelTitleBarTools {
  QWidget* widget = nullptr;
  QToolButton* settings_button = nullptr;
  QToolButton* expand_button = nullptr;
};

PanelTitleBarTools CreateRvizPanelTitleBarTools(
    QWidget* parent, const PanelContextMenuCallbacks& callbacks,
    const PanelTitleBarOptions& options = {});

/** Expand + overflow menu title bar tools. */
QWidget* CreateStandardPanelTitleTools(
    QWidget* parent, const PanelContextMenuCallbacks& callbacks);

}  // namespace autoviz
