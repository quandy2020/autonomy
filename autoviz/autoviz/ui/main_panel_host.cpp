/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/main_panel_host.hpp"

#include <QDockWidget>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QTabWidget>
#include <QWidget>

namespace autoviz {

MainPanelHost::MainPanelHost(QWidget* parent) : QMainWindow(parent) {
  setObjectName(QStringLiteral("MainPanelHost"));
  setWindowFlags(Qt::Widget);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setDockOptions(QMainWindow::AnimatedDocks | QMainWindow::AllowNestedDocks |
                 QMainWindow::AllowTabbedDocks);
  setTabPosition(Qt::AllDockWidgetAreas, QTabWidget::North);
  setDockNestingEnabled(true);
  setContentsMargins(0, 0, 0, 0);

  auto* central = new QWidget(this);
  central->setMinimumSize(0, 0);
  central->setMaximumSize(0, 0);
  central->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  setCentralWidget(central);
}

void MainPanelHost::syncHorizontalDockLayout() {
  QList<QDockWidget*> horizontal_docks;
  horizontal_docks.reserve(4);
  for (QDockWidget* dock : findChildren<QDockWidget*>()) {
    if (dock == nullptr || dock->isFloating() || !dock->isVisible()) {
      continue;
    }
    const Qt::DockWidgetArea area = dockWidgetArea(dock);
    if (area == Qt::LeftDockWidgetArea || area == Qt::RightDockWidgetArea) {
      horizontal_docks.push_back(dock);
    }
  }
  if (horizontal_docks.isEmpty()) {
    return;
  }

  const int total_width = qMax(width(), 1);
  if (horizontal_docks.size() == 1) {
    resizeDocks(horizontal_docks, {total_width}, Qt::Horizontal);
    return;
  }

  const int per_dock = qMax(total_width / horizontal_docks.size(), 120);
  QList<int> sizes;
  sizes.reserve(horizontal_docks.size());
  int used = 0;
  for (int i = 0; i < horizontal_docks.size(); ++i) {
    const int size =
        (i == horizontal_docks.size() - 1) ? qMax(total_width - used, 120) : per_dock;
    sizes.push_back(size);
    used += size;
  }
  resizeDocks(horizontal_docks, sizes, Qt::Horizontal);
}

void MainPanelHost::resizeEvent(QResizeEvent* event) {
  QMainWindow::resizeEvent(event);
  syncHorizontalDockLayout();
}

}  // namespace autoviz
