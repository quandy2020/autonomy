/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>

class QFrame;
class QIcon;
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

QFrame* CreateTitleSeparator(QWidget* parent);

/** Expand + overflow menu title bar tools. */
QWidget* CreateStandardPanelTitleTools(
    QWidget* parent, const PanelContextMenuCallbacks& callbacks);

}  // namespace autoviz
