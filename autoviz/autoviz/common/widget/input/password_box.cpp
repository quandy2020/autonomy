/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/input/password_box.hpp"

#include <QAction>
#include <QEvent>
#include <QKeySequence>
#include <QPaintEvent>
#include <QPainter>
#include <QPalette>

#include "autoviz/common/widget/foundation/semantic_icon.hpp"
#include "autoviz/common/widget/foundation/validation_state.hpp"

namespace autoviz::widget {
namespace {

void EnsureTransparentEditorBase(QLineEdit* editor) {
  QPalette palette = editor->palette();
  bool changed = false;
  for (const QPalette::ColorGroup group :
       {QPalette::Active, QPalette::Inactive, QPalette::Disabled}) {
    if (palette.brush(group, QPalette::Base).color().alpha() != 0) {
      palette.setBrush(group, QPalette::Base, Qt::transparent);
      changed = true;
    }
  }
  if (changed) {
    editor->setPalette(palette);
  }
}

}  // namespace

PasswordBox::PasswordBox(QWidget* parent) : QLineEdit(parent) {
  setObjectName(QStringLiteral("AutovizPasswordBox"));
  setProperty("autovizRole", QStringLiteral("passwordBox"));
  setAccessibleName(tr("Password"));
  setEchoMode(QLineEdit::Password);
  setFrame(false);
  setFocusPolicy(Qt::StrongFocus);
  setAttribute(Qt::WA_Hover);
  setTextMargins(10, 0, 4, 0);
  setAttribute(Qt::WA_TranslucentBackground);
  EnsureTransparentEditorBase(this);

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
  result.setHeight(GetThemeTokens(this, tone_).control_height_medium);
  result.setWidth(qMax(result.width(), 200));
  return result;
}

bool PasswordBox::event(QEvent* event) {
  switch (event->type()) {
    case QEvent::Enter:
    case QEvent::Leave:
    case QEvent::FocusIn:
    case QEvent::FocusOut:
      update();
      break;
    case QEvent::EnabledChange:
      if (reveal_action_ != nullptr) {
        UpdateRevealAction();
      }
      update();
      break;
    case QEvent::PaletteChange:
      EnsureTransparentEditorBase(this);
      if (reveal_action_ != nullptr) {
        UpdateRevealAction();
      }
      update();
      break;
    case QEvent::DynamicPropertyChange:
      update();
      break;
    default:
      break;
  }
  return QLineEdit::event(event);
}

void PasswordBox::paintEvent(QPaintEvent* event) {
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  QColor fill = tokens.surface_sunken;
  QColor border = tokens.border_subtle;
  qreal border_width = 1.0;
  if (!isEnabled()) {
    fill = tokens.surface_sunken;
    border = tokens.disabled;
  } else if (internal::HasValidationError(this)) {
    border = tokens.danger;
    border_width = hasFocus() ? tokens.focus_ring_width + 1.0 : 2.0;
  } else if (hasFocus()) {
    border = tokens.focus_ring;
    border_width = tokens.focus_ring_width;
  } else if (underMouse()) {
    fill = tokens.surface_material;
  }

  QPainter background(this);
  background.setRenderHint(QPainter::Antialiasing);
  background.setPen(Qt::NoPen);
  background.setBrush(fill);
  background.drawRoundedRect(QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5),
                             tokens.radius_small, tokens.radius_small);
  background.end();

  QLineEdit::paintEvent(event);

  QPainter outline(this);
  outline.setRenderHint(QPainter::Antialiasing);
  outline.setBrush(Qt::NoBrush);
  outline.setPen(
      QPen(border, border_width, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  const qreal inset = border_width * 0.5;
  outline.drawRoundedRect(QRectF(rect()).adjusted(inset, inset, -inset, -inset),
                          tokens.radius_small, tokens.radius_small);
}

void PasswordBox::UpdateRevealAction() {
  reveal_action_->setText(password_visible_ ? tr("Hide password")
                                            : tr("Show password"));
  reveal_action_->setToolTip(password_visible_ ? tr("Hide password")
                                               : tr("Show password"));
  reveal_action_->setIcon(
      internal::MakePaletteVisibilityIcon(palette(), password_visible_));
}

}  // namespace autoviz::widget
