/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel/title_tools.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QMenu>
#include <QTimer>
#include <QToolButton>
#include <QWidgetAction>

#include "autoviz/ui/panel/change_menu.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/panel/context_menu.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {

namespace {

void WireExpandButton(QToolButton* button, const std::function<void()>& on_expand) {
  if (button == nullptr || !on_expand) {
    return;
  }
  QObject::connect(button, &QToolButton::clicked, button,
                   [on_expand]() { on_expand(); });
}

void AddChangePanelButton(QWidget* tools, QHBoxLayout* layout,
                          const PanelContextMenuCallbacks& callbacks) {
  auto* change_button = CreateTitleToolButton(
      tools, IconLoader::menuIcon(QStringLiteral("panels.add")),
      QObject::tr("Change panel"));
  auto* change_menu = new QMenu(change_button);
  auto* picker = new ChangePanelMenuWidget(change_menu);
  auto* picker_action = new QWidgetAction(change_menu);
  picker_action->setDefaultWidget(picker);
  change_menu->addAction(picker_action);
  QObject::connect(picker, &ChangePanelMenuWidget::panelSelected, change_button,
                   [callbacks, change_menu](const QString& object_name) {
                     // Close the popup first — changing docks while the menu is
                     // still open leaves it covering the new panel toolbar.
                     change_menu->close();
                     if (object_name == callbacks.current_object_name) {
                       return;
                     }
                     if (!callbacks.change_panel) {
                       return;
                     }
                     const auto change = callbacks.change_panel;
                     QTimer::singleShot(0, change_menu, [change, object_name]() {
                       change(object_name);
                     });
                   });
  ConfigurePanelMoreToolButton(change_button, change_menu);
  layout->addWidget(change_button);
}

}  // namespace

QString PanelTitleToolsStyleSheet() {
  return style::sheet(QStringLiteral("chrome/title_tools"), style::tokens());
}

QString PlotTitleToolsStyleSheet() {
  return style::sheet(QStringLiteral("chrome/title_tools"), style::tokens());
}

QToolButton* CreateTitleToolButton(QWidget* parent, const QIcon& icon,
                                   const QString& tip, bool checkable) {
  auto* button = new QToolButton(parent);
  button->setIcon(icon);
  button->setIconSize(QSize(16, 16));
  button->setAutoRaise(true);
  button->setToolTip(tip);
  button->setCheckable(checkable);
  button->setFixedSize(QSize(24, 22));
  return button;
}

QToolButton* CreatePlotTitleToolButton(QWidget* parent, const QIcon& icon,
                                       const QString& tip, bool checkable) {
  auto* button = new QToolButton(parent);
  button->setIcon(icon);
  button->setIconSize(QSize(18, 18));
  button->setAutoRaise(true);
  button->setToolTip(tip);
  button->setCheckable(checkable);
  button->setFixedSize(QSize(26, 24));
  return button;
}

void ConfigurePanelMoreToolButton(QToolButton* button, QMenu* menu) {
  if (button == nullptr) {
    return;
  }
  button->setToolButtonStyle(Qt::ToolButtonIconOnly);
  button->setPopupMode(QToolButton::InstantPopup);
  // macOS still paints a native caret unless the indicator is zeroed on the
  // button itself (parent stylesheet alone is not enough).
  button->setStyleSheet(
      style::sheet(QStringLiteral("widget/menu_indicator_hidden")));
  if (menu != nullptr) {
    button->setMenu(menu);
  }
}

QFrame* CreateTitleSeparator(QWidget* parent) {
  auto* separator = new QFrame(parent);
  separator->setFrameShape(QFrame::VLine);
  separator->setFrameShadow(QFrame::Plain);
  separator->setFixedSize(QSize(1, 18));
  separator->setStyleSheet(
      style::sheet(QStringLiteral("widget/separator_mid")));
  return separator;
}

PanelTitleBarTools CreatePanelTitleBarTools(
    QWidget* parent, const PanelContextMenuCallbacks& callbacks,
    const PanelTitleBarOptions& options) {
  auto* tools = new QWidget(parent);
  ApplyPanelTitleToolsChrome(tools);
  auto* layout = new QHBoxLayout(tools);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  PanelTitleBarTools result;
  result.widget = tools;

  if (options.show_reset && options.on_reset) {
    auto* reset_button = CreateTitleToolButton(
        tools, IconLoader::panelTitleIcon(QStringLiteral("plot.reset_view")),
        QObject::tr("Reset view"));
    layout->addWidget(reset_button);
    QObject::connect(reset_button, &QToolButton::clicked, tools,
                     [on_reset = options.on_reset]() { on_reset(); });
  }

  if (options.show_settings && options.on_settings_toggled) {
    result.settings_button = CreateTitleToolButton(
        tools, IconLoader::panelTitleIcon(QStringLiteral("panel.settings")),
        QObject::tr("Settings"), true);
    result.settings_button->setChecked(options.settings_checked);
    layout->addWidget(result.settings_button);
    QObject::connect(result.settings_button, &QToolButton::toggled, tools,
                     options.on_settings_toggled);
  }

  if (options.show_split) {
    auto* split_right = CreateTitleToolButton(
        tools, IconLoader::panelTitleIcon(QStringLiteral("panel.split_right")),
        QObject::tr("Split right"));
    layout->addWidget(split_right);
    QObject::connect(split_right, &QToolButton::clicked, tools, [callbacks]() {
      if (callbacks.split) {
        callbacks.split(Qt::Horizontal);
      }
    });

    auto* split_down = CreateTitleToolButton(
        tools, IconLoader::panelTitleIcon(QStringLiteral("panel.split_down")),
        QObject::tr("Split down"));
    layout->addWidget(split_down);
    QObject::connect(split_down, &QToolButton::clicked, tools, [callbacks]() {
      if (callbacks.split) {
        callbacks.split(Qt::Vertical);
      }
    });
  }

  if (options.show_change) {
    AddChangePanelButton(tools, layout, callbacks);
  }

  if (callbacks.download_plot_csv) {
    auto* download = CreateTitleToolButton(
        tools, IconLoader::menuIcon(QStringLiteral("file.save")),
        QObject::tr("Download plot data as CSV"));
    layout->addWidget(download);
    QObject::connect(download, &QToolButton::clicked, tools, [callbacks]() {
      if (callbacks.download_plot_csv) {
        callbacks.download_plot_csv();
      }
    });
  }
  if (callbacks.download_plot_png) {
    auto* download_png = CreateTitleToolButton(
        tools, IconLoader::menuIcon(QStringLiteral("file.image")),
        QObject::tr("Download plot as PNG"));
    layout->addWidget(download_png);
    QObject::connect(download_png, &QToolButton::clicked, tools, [callbacks]() {
      if (callbacks.download_plot_png) {
        callbacks.download_plot_png();
      }
    });
  }
  if (callbacks.download_image_png) {
    auto* download_image = CreateTitleToolButton(
        tools, IconLoader::menuIcon(QStringLiteral("file.image")),
        QObject::tr("Download image as PNG"));
    layout->addWidget(download_image);
    QObject::connect(download_image, &QToolButton::clicked, tools,
                     [callbacks]() {
                       if (callbacks.download_image_png) {
                         callbacks.download_image_png();
                       }
                     });
  }

  if (options.show_expand) {
    result.expand_button = CreateTitleToolButton(
        tools, IconLoader::panelExpandIcon(),
        QObject::tr("Expand"), options.expand_checkable);
    layout->addWidget(result.expand_button);
    WireExpandButton(result.expand_button, options.on_expand
                                               ? options.on_expand
                                               : callbacks.expand);
  }

  return result;
}

QWidget* CreateStandardPanelTitleTools(
    QWidget* parent, const PanelContextMenuCallbacks& callbacks) {
  PanelTitleBarOptions options;
  options.show_expand = true;
  options.expand_checkable = false;
  options.on_expand = callbacks.expand;
  return CreatePanelTitleBarTools(parent, callbacks, options).widget;
}

}  // namespace autoviz
