/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel_context_menu.hpp"

#include <QCoreApplication>
#include <QMenu>
#include <QWidgetAction>

#include "autoviz/ui/change_panel_menu_widget.hpp"

namespace autoviz {

QMenu* CreatePanelContextMenu(QWidget* parent,
                              const PanelContextMenuCallbacks& callbacks) {
  auto* menu = new QMenu(parent);

  auto* change_menu = menu->addMenu(QCoreApplication::translate("autoviz", "Change panel"));
  auto* picker = new ChangePanelMenuWidget(change_menu);
  auto* picker_action = new QWidgetAction(change_menu);
  picker_action->setDefaultWidget(picker);
  change_menu->addAction(picker_action);
  QObject::connect(
      picker, &ChangePanelMenuWidget::panelSelected, change_menu,
      [callbacks, change_menu, menu](const QString& object_name) {
        if (object_name == callbacks.current_object_name) {
          change_menu->close();
          return;
        }
        if (callbacks.change_panel) {
          callbacks.change_panel(object_name);
        }
        change_menu->close();
        if (menu->parentWidget() != nullptr) {
          menu->close();
        }
      });

  menu->addSeparator();

  auto* split_right =
      menu->addAction(QCoreApplication::translate("autoviz", "Split right"));
  QObject::connect(split_right, &QAction::triggered, menu, [callbacks]() {
    if (callbacks.split) {
      callbacks.split(Qt::Horizontal);
    }
  });

  auto* split_down =
      menu->addAction(QCoreApplication::translate("autoviz", "Split down"));
  QObject::connect(split_down, &QAction::triggered, menu, [callbacks]() {
    if (callbacks.split) {
      callbacks.split(Qt::Vertical);
    }
  });

  auto* expand = menu->addAction(QCoreApplication::translate("autoviz", "Expand"));
  QObject::connect(expand, &QAction::triggered, menu, [callbacks]() {
    if (callbacks.expand) {
      callbacks.expand();
    }
  });

  if (callbacks.download_plot_csv) {
    menu->addSeparator();
    auto* download_csv = menu->addAction(
        QCoreApplication::translate("autoviz", "Download plot data as CSV"));
    QObject::connect(download_csv, &QAction::triggered, menu, [callbacks]() {
      if (callbacks.download_plot_csv) {
        callbacks.download_plot_csv();
      }
    });
  }

  menu->addSeparator();

  auto* remove =
      menu->addAction(QCoreApplication::translate("autoviz", "Remove panel"));
  QObject::connect(remove, &QAction::triggered, menu, [callbacks]() {
    if (callbacks.remove) {
      callbacks.remove();
    }
  });

  return menu;
}

}  // namespace autoviz
