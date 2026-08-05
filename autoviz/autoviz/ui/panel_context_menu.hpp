/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <Qt>

#include <functional>

#include <QString>

class QMenu;
class QWidget;

namespace autoviz {

struct PanelContextMenuCallbacks {
  QString current_object_name;
  std::function<void(const QString& object_name)> change_panel;
  std::function<void(Qt::Orientation orientation)> split;
  std::function<void()> expand;
  std::function<void()> remove;
  /** When set, adds "Download plot data as CSV" (Plot panel). */
  std::function<void()> download_plot_csv;
};

/** Foxglove-style panel overflow menu (Change panel, Split, Expand, Remove). */
QMenu* CreatePanelContextMenu(QWidget* parent,
                              const PanelContextMenuCallbacks& callbacks);

}  // namespace autoviz
