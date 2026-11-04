/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/display/tag.hpp"

#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <algorithm>

namespace autoviz::widget {
namespace {

QColor Mix(const QColor& foreground, const QColor& background, int alpha) {
  const int bounded = std::clamp(alpha, 0, 255);
  const int inverse = 255 - bounded;
  return QColor(
      (foreground.red() * bounded + background.red() * inverse) / 255,
      (foreground.green() * bounded + background.green() * inverse) / 255,
      (foreground.blue() * bounded + background.blue() * inverse) / 255);
}

QColor AccentForKind(Tag::Kind kind, const ThemeTokens& tokens) {
  switch (kind) {
    case Tag::Kind::kNeutral:
      return tokens.text_muted;
    case Tag::Kind::kAccent:
      return tokens.accent;
    case Tag::Kind::kSuccess:
      return tokens.success;
    case Tag::Kind::kWarning:
      return tokens.warning;
    case Tag::Kind::kDanger:
      return tokens.danger;
  }
  return tokens.text_muted;
}

}  // namespace

Tag::Tag(QWidget* parent) : QAbstractButton(parent) {
  setObjectName(QStringLiteral("AutovizTag"));
  setProperty("autovizRole", QStringLiteral("tag"));
  setFocusPolicy(Qt::StrongFocus);
  setCursor(Qt::PointingHandCursor);
  setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  UpdateAccessibility();
}

Tag::Tag(const QString& text, QWidget* parent) : Tag(parent) {
  setText(text);
  setAccessibleName(text);
}

void Tag::SetKind(Kind kind) {
  if (kind_ == kind) {
    return;
  }
  kind_ = kind;
  update();
}

void Tag::SetRemovable(bool removable) {
  if (removable_ == removable) {
    return;
  }
  removable_ = removable;
  UpdateAccessibility();
  updateGeometry();
  update();
}

void Tag::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  update();
}

QSize Tag::sizeHint() const {
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  const int close_width = removable_ ? tokens.control_height_medium : 0;
  return QSize(fontMetrics().horizontalAdvance(text()) + close_width +
                   2 * tokens.spacing_md,
               tokens.control_height_medium);
}

QSize Tag::minimumSizeHint() const {
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  return QSize(tokens.control_height_medium, tokens.control_height_medium);
}

void Tag::keyPressEvent(QKeyEvent* event) {
  if (removable_ &&
      (event->key() == Qt::Key_Delete || event->key() == Qt::Key_Backspace)) {
    emit RemoveRequested();
    event->accept();
    return;
  }
  QAbstractButton::keyPressEvent(event);
}

void Tag::mouseReleaseEvent(QMouseEvent* event) {
  if (removable_ && event->button() == Qt::LeftButton &&
      RemoveRect().contains(event->position().toPoint())) {
    setDown(false);
    emit RemoveRequested();
    event->accept();
    return;
  }
  QAbstractButton::mouseReleaseEvent(event);
}

void Tag::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  QColor accent = AccentForKind(kind_, tokens);
  if (!isEnabled()) {
    accent = tokens.disabled;
  }
  const QColor background =
      isChecked() ? Mix(accent, tokens.surface, 54)
                  : Mix(accent, tokens.surface, underMouse() ? 34 : 22);
  const QColor text_color = isEnabled() ? tokens.text : tokens.text_secondary;

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(QPen(hasFocus() ? tokens.focus_ring : accent,
                      hasFocus() ? tokens.focus_ring_width : 1));
  painter.setBrush(background);
  const QRect capsule_rect(0, (height() - tokens.control_height_small) / 2,
                           width(), tokens.control_height_small);
  painter.drawRoundedRect(QRectF(capsule_rect).adjusted(0.5, 0.5, -0.5, -0.5),
                          capsule_rect.height() / 2.0,
                          capsule_rect.height() / 2.0);

  QRect text_rect =
      capsule_rect.adjusted(tokens.spacing_md, 0, -tokens.spacing_md, 0);
  if (removable_) {
    if (layoutDirection() == Qt::RightToLeft) {
      text_rect.setLeft(RemoveRect().right() + tokens.spacing_xs);
    } else {
      text_rect.setRight(RemoveRect().left() - tokens.spacing_xs);
    }
  }
  painter.setPen(text_color);
  const Qt::TextElideMode elide_mode =
      layoutDirection() == Qt::RightToLeft ? Qt::ElideLeft : Qt::ElideRight;
  painter.drawText(
      text_rect, Qt::AlignVCenter | Qt::AlignLeading | Qt::TextSingleLine,
      fontMetrics().elidedText(text(), elide_mode, text_rect.width()));

  if (removable_) {
    QRect close(QPoint(), QSize(tokens.spacing_md, tokens.spacing_md));
    close.moveCenter(RemoveRect().center());
    painter.drawLine(close.topLeft(), close.bottomRight());
    painter.drawLine(close.topRight(), close.bottomLeft());
  }
}

QRect Tag::RemoveRect() const {
  const int extent = GetThemeTokens(this, tone_).control_height_medium;
  const int x = layoutDirection() == Qt::RightToLeft ? 0 : width() - extent;
  return QRect(x, (height() - extent) / 2, extent, extent);
}

void Tag::UpdateAccessibility() {
  setAccessibleDescription(removable_ ? tr("Removable tag") : tr("Tag"));
}

}  // namespace autoviz::widget
