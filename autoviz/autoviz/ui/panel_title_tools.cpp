/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel_title_tools.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QToolButton>

#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/panel_context_menu.hpp"

namespace autoviz {

QString PanelTitleToolsStyleSheet() {
  return QStringLiteral(
      "QToolButton { border: none; margin: 0 1px; padding: 2px; color: #444444; }"
      "QToolButton:hover { background: rgba(0,0,0,0.06); border-radius: 3px; }"
      "QToolButton:checked { background: #555555; border-radius: 3px; color: #ffffff; }"
      "QToolButton:checked:hover { background: #666666; }");
}

QString PlotTitleToolsStyleSheet() {
  return QStringLiteral(
      "QToolButton {"
      "  border: 1px solid transparent;"
      "  margin: 0 1px;"
      "  padding: 1px;"
      "  background: transparent;"
      "  border-radius: 4px;"
      "}"
      "QToolButton:hover {"
      "  background: rgba(25, 118, 210, 0.12);"
      "  border-color: rgba(25, 118, 210, 0.35);"
      "}"
      "QToolButton:pressed {"
      "  background: rgba(25, 118, 210, 0.2);"
      "}"
      "QToolButton:checked {"
      "  background: rgba(25, 118, 210, 0.22);"
      "  border-color: #1976D2;"
      "}"
      "QToolButton:checked:hover {"
      "  background: rgba(25, 118, 210, 0.3);"
      "}");
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

QFrame* CreateTitleSeparator(QWidget* parent) {
  auto* separator = new QFrame(parent);
  separator->setFrameShape(QFrame::VLine);
  separator->setFrameShadow(QFrame::Plain);
  separator->setFixedSize(QSize(1, 18));
  separator->setStyleSheet(QStringLiteral("color: #b0bec5;"));
  return separator;
}

QWidget* CreateStandardPanelTitleTools(
    QWidget* parent, const PanelContextMenuCallbacks& callbacks) {
  auto* tools = new QWidget(parent);
  tools->setStyleSheet(PanelTitleToolsStyleSheet());
  auto* layout = new QHBoxLayout(tools);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  auto* expand_button = CreateTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_fullscreen.svg")),
      QObject::tr("Expand"));
  layout->addWidget(expand_button);
  if (callbacks.expand) {
    QObject::connect(expand_button, &QToolButton::clicked, tools,
                     [callbacks]() { callbacks.expand(); });
  }

  auto* more_button = CreateTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_more.svg")),
      QObject::tr("More"));
  more_button->setPopupMode(QToolButton::InstantPopup);
  more_button->setMenu(CreatePanelContextMenu(more_button, callbacks));
  layout->addWidget(more_button);

  return tools;
}

}  // namespace autoviz
