/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/navigation/command_bar.hpp"

#include <QAction>

#include "autoviz/common/widget/foundation/widget_theme.hpp"

namespace autoviz::widget {

CommandBar::CommandBar(QWidget *parent) : QToolBar(parent) {
  setObjectName(QStringLiteral("AutovizCommandBar"));
  setProperty("autovizRole", QStringLiteral("commandBar"));
  setAccessibleName(tr("Command bar"));
  setMovable(false);
  setFloatable(false);
  SetDisplayMode(display_mode_);
  ApplyTheme();
}

QAction *CommandBar::AddCommand(const QString &text, const QIcon &icon) {
  QAction *action = addAction(icon, text);
  action->setToolTip(text);
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
}

void CommandBar::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  ApplyTheme();
}

void CommandBar::ApplyTheme() {
  const ThemeTokens &tokens = GetThemeTokens(tone_);
  setIconSize(QSize(tokens.control_height_small - tokens.spacing_sm,
                    tokens.control_height_small - tokens.spacing_sm));
  ApplyWidgetTheme(this, tone_, true);
}

}  // namespace autoviz::widget
