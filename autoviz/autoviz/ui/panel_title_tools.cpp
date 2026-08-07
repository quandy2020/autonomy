/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel_title_tools.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QMenu>
#include <QToolButton>

#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {

namespace {

QString UnifiedTitleToolsStyleSheet() {
  return QStringLiteral(
      "QToolButton {"
      "  border: 1px solid transparent;"
      "  margin: 0 1px;"
      "  padding: 1px;"
      "  background: transparent;"
      "  border-radius: 4px;"
      "  color: palette(text);"
      "}"
      "QToolButton:hover {"
      "  background: palette(midlight);"
      "  border-color: palette(mid);"
      "}"
      "QToolButton:pressed {"
      "  background: palette(mid);"
      "}"
      "QToolButton:checked {"
      "  background: palette(highlight);"
      "  color: palette(highlighted-text);"
      "  border-color: palette(highlight);"
      "}"
      "QToolButton:checked:hover {"
      "  background: palette(highlight);"
      "}"
      "QToolButton::menu-indicator {"
      "  image: none;"
      "  width: 0px;"
      "  height: 0px;"
      "}");
}

void WireExpandButton(QToolButton* button, const std::function<void()>& on_expand) {
  if (button == nullptr || !on_expand) {
    return;
  }
  QObject::connect(button, &QToolButton::clicked, button,
                   [on_expand]() { on_expand(); });
}

}  // namespace

QString PanelTitleToolsStyleSheet() {
  return UnifiedTitleToolsStyleSheet();
}

QString PlotTitleToolsStyleSheet() {
  return UnifiedTitleToolsStyleSheet();
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
  if (menu != nullptr) {
    button->setMenu(menu);
  }
}

QFrame* CreateTitleSeparator(QWidget* parent) {
  auto* separator = new QFrame(parent);
  separator->setFrameShape(QFrame::VLine);
  separator->setFrameShadow(QFrame::Plain);
  separator->setFixedSize(QSize(1, 18));
  separator->setStyleSheet(QStringLiteral("color: palette(mid);"));
  return separator;
}

PanelTitleBarTools CreateRvizPanelTitleBarTools(
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

  if (options.show_expand) {
    result.expand_button = CreateTitleToolButton(
        tools, IconLoader::panelExpandIcon(),
        QObject::tr("Expand"), options.expand_checkable);
    layout->addWidget(result.expand_button);
    WireExpandButton(result.expand_button, options.on_expand);
  }

  if (options.show_more) {
    auto* more_button = CreateTitleToolButton(
        tools, IconLoader::panelTitleIcon(QStringLiteral("panel.more")),
        QObject::tr("More"));
    ConfigurePanelMoreToolButton(
        more_button, CreatePanelContextMenu(more_button, callbacks));
    layout->addWidget(more_button);
  }

  return result;
}

QWidget* CreateStandardPanelTitleTools(
    QWidget* parent, const PanelContextMenuCallbacks& callbacks) {
  PanelTitleBarOptions options;
  options.show_expand = true;
  options.expand_checkable = false;
  options.on_expand = callbacks.expand;
  return CreateRvizPanelTitleBarTools(parent, callbacks, options).widget;
}

}  // namespace autoviz
