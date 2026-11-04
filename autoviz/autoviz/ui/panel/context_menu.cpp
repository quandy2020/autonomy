/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel/context_menu.hpp"

#include <QAction>
#include <QApplication>
#include <QCoreApplication>
#include <QMenu>
#include <QTimer>

#include "autoviz/ui/panel/change_menu.hpp"

namespace autoviz {

QMenu* CreatePanelContextMenu(QWidget* parent,
                              const PanelContextMenuCallbacks& callbacks) {
  auto* menu = new QMenu(parent);

  auto* change_panel = menu->addAction(
      QCoreApplication::translate("autoviz", "Change panel"));
  QObject::connect(
      change_panel, &QAction::triggered, menu,
      [callbacks, menu]() {
        QWidget* anchor = menu->parentWidget();
        if (anchor == nullptr) {
          anchor = QApplication::activeWindow();
        }
        // Close the context menu first so it cannot cover the picker.
        menu->close();
        ShowChangePanelPopup(
            anchor, [callbacks](const QString& object_name) {
              if (object_name == callbacks.current_object_name) {
                return;
              }
              if (!callbacks.change_panel) {
                return;
              }
              const auto change = callbacks.change_panel;
              QTimer::singleShot(0, qApp, [change, object_name]() {
                change(object_name);
              });
            });
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
  if (callbacks.download_plot_png) {
    if (!callbacks.download_plot_csv) {
      menu->addSeparator();
    }
    auto* download_png = menu->addAction(
        QCoreApplication::translate("autoviz", "Download plot as PNG"));
    QObject::connect(download_png, &QAction::triggered, menu, [callbacks]() {
      if (callbacks.download_plot_png) {
        callbacks.download_plot_png();
      }
    });
  }
  if (callbacks.download_image_png) {
    menu->addSeparator();
    auto* download_image = menu->addAction(
        QCoreApplication::translate("autoviz", "Download image as PNG"));
    QObject::connect(download_image, &QAction::triggered, menu, [callbacks]() {
      if (callbacks.download_image_png) {
        callbacks.download_image_png();
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
