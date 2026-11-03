/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/input/password_box.hpp"

#include <QAction>
#include <QIcon>
#include <QKeySequence>
#include <QPaintEvent>
#include <QPainter>
#include <QPalette>
#include <QStyle>

namespace autoviz::widget {

PasswordBox::PasswordBox(QWidget* parent) : QLineEdit(parent) {
  setObjectName(QStringLiteral("AutovizPasswordBox"));
  setProperty("autovizRole", QStringLiteral("passwordBox"));
  setAccessibleName(tr("Password"));
  setEchoMode(QLineEdit::Password);
  setFrame(false);
  setFocusPolicy(Qt::StrongFocus);
  setTextMargins(10, 0, 4, 0);
  QPalette field_palette = palette();
  field_palette.setBrush(QPalette::Base, Qt::transparent);
  setPalette(field_palette);
  setAttribute(Qt::WA_TranslucentBackground);

  reveal_action_ = new QAction(this);
  addAction(reveal_action_, QLineEdit::TrailingPosition);
  reveal_action_->setCheckable(true);
  reveal_action_->setShortcut(QKeySequence(Qt::ALT | Qt::Key_V));
  reveal_action_->setShortcutContext(Qt::WidgetShortcut);
  connect(reveal_action_, &QAction::triggered, this,
          [this](bool visible) { SetPasswordVisible(visible); });
  UpdateRevealAction();
}

void PasswordBox::SetPasswordVisible(bool visible) {
  if (password_visible_ == visible) {
    return;
  }
  password_visible_ = visible;
  setEchoMode(visible ? QLineEdit::Normal : QLineEdit::Password);
  reveal_action_->setChecked(visible);
  UpdateRevealAction();
}

void PasswordBox::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  update();
}

QSize PasswordBox::sizeHint() const {
  QSize result = QLineEdit::sizeHint();
  result.setHeight(GetThemeTokens(tone_).control_height_medium);
  result.setWidth(qMax(result.width(), 200));
  return result;
}

void PasswordBox::paintEvent(QPaintEvent* event) {
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setBrush(isEnabled() ? tokens.surface_raised : tokens.surface);
  painter.setPen(QPen(hasFocus() ? tokens.focus_ring : tokens.border,
                      hasFocus() ? tokens.focus_ring_width : 1));
  const qreal inset = hasFocus() ? tokens.focus_ring_width * 0.5 : 0.5;
  painter.drawRoundedRect(QRectF(rect()).adjusted(inset, inset, -inset, -inset),
                          tokens.radius_small, tokens.radius_small);
  painter.end();
  QLineEdit::paintEvent(event);
}

void PasswordBox::UpdateRevealAction() {
  reveal_action_->setText(password_visible_ ? tr("Hide password")
                                            : tr("Show password"));
  reveal_action_->setToolTip(password_visible_ ? tr("Hide password")
                                               : tr("Show password"));
  const QString icon_name = password_visible_
                                ? QStringLiteral("view-conceal-symbolic")
                                : QStringLiteral("view-reveal-symbolic");
  reveal_action_->setIcon(QIcon::fromTheme(
      icon_name, style()->standardIcon(QStyle::SP_TitleBarContextHelpButton)));
}

}  // namespace autoviz::widget
