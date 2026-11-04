/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/navigation/command_bar.hpp"

#include <QAction>
#include <QLayout>
#include <QPaintEvent>
#include <QPainter>
#include <QToolButton>

#include "autoviz/common/widget/foundation/widget_theme.hpp"

namespace autoviz::widget {

CommandBar::CommandBar(QWidget *parent) : QToolBar(parent) {
  setObjectName(QStringLiteral("AutovizCommandBar"));
  setProperty("autovizRole", QStringLiteral("commandBar"));
  setAccessibleName(tr("Command bar"));
  setAttribute(Qt::WA_TranslucentBackground);
  setAutoFillBackground(false);
  setMovable(false);
  setFloatable(false);
  SetDisplayMode(display_mode_);
  ApplyTheme();
}

QAction *CommandBar::AddCommand(const QString &text, const QIcon &icon) {
  QAction *action = addAction(icon, text);
  action->setToolTip(text);
  SyncActionButtonMetrics();
  return action;
}

void CommandBar::SetDisplayMode(DisplayMode mode) {
  display_mode_ = mode;
  switch (display_mode_) {
    case DisplayMode::kIconOnly:
      setToolButtonStyle(Qt::ToolButtonIconOnly);
      break;
    case DisplayMode::kTextBesideIcon:
      setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
      break;
    case DisplayMode::kTextOnly:
      setToolButtonStyle(Qt::ToolButtonTextOnly);
      break;
  }
  SyncActionButtonMetrics();
}

void CommandBar::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  ApplyTheme();
}

void CommandBar::paintEvent(QPaintEvent *event) {
  Q_UNUSED(event);
  const ThemeTokens &tokens = GetThemeTokens(this, tone_);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setBrush(tokens.surface_raised);
  painter.setPen(QPen(tokens.border, 1.0));
  painter.drawRoundedRect(QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5),
                          tokens.radius_medium, tokens.radius_medium);
}

void CommandBar::ApplyTheme() {
  const ThemeTokens &tokens = GetThemeTokens(this, tone_);
  setIconSize(QSize(tokens.control_height_small - tokens.spacing_sm,
                    tokens.control_height_small - tokens.spacing_sm));
  setContentsMargins(tokens.spacing_xs, tokens.spacing_xs, tokens.spacing_xs,
                     tokens.spacing_xs);
  setMinimumHeight(tokens.control_height_medium + 2 * tokens.spacing_xs);
  if (layout() != nullptr) {
    layout()->setSpacing(tokens.spacing_xs);
  }
  ApplyWidgetTheme(this, tone_, true);
  SyncActionButtonMetrics();
  update();
}

void CommandBar::SyncActionButtonMetrics() {
  const int minimum_extent = GetThemeTokens(this, tone_).control_height_medium;
  const QSize minimum_size(minimum_extent, minimum_extent);
  for (QAction *action : actions()) {
    auto *button = qobject_cast<QToolButton *>(widgetForAction(action));
    if (button == nullptr) {
      continue;
    }
    button->setMinimumSize(button->minimumSize().expandedTo(minimum_size));
    button->updateGeometry();
  }
}

}  // namespace autoviz::widget
