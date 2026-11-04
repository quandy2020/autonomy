/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/input/rating.hpp"

#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QtMath>
#include <algorithm>

namespace autoviz::widget {
namespace {

QPainterPath StarPath(const QPointF& center, qreal radius) {
  QPainterPath path;
  constexpr int kPoints = 10;
  for (int point = 0; point < kPoints; ++point) {
    const qreal angle = qDegreesToRadians(-90.0 + point * 36.0);
    const qreal point_radius = point % 2 == 0 ? radius : radius * 0.44;
    const QPointF position(center.x() + qCos(angle) * point_radius,
                           center.y() + qSin(angle) * point_radius);
    if (point == 0) {
      path.moveTo(position);
    } else {
      path.lineTo(position);
    }
  }
  path.closeSubpath();
  return path;
}

}  // namespace

Rating::Rating(QWidget* parent) : QAbstractButton(parent) {
  setObjectName(QStringLiteral("AutovizRating"));
  setProperty("autovizRole", QStringLiteral("rating"));
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
  setCursor(Qt::PointingHandCursor);
  setAccessibleName(tr("Rating"));
  UpdateAccessibility();
}

void Rating::SetValue(int value) {
  const int bounded = qBound(0, value, maximum_);
  if (value_ == bounded) {
    return;
  }
  value_ = bounded;
  UpdateAccessibility();
  update();
  emit ValueChanged(value_);
}

void Rating::SetMaximum(int maximum) {
  const int bounded = qBound(1, maximum, 20);
  if (maximum_ == bounded) {
    return;
  }
  const int previous_value = value_;
  maximum_ = bounded;
  value_ = qMin(value_, maximum_);
  hover_value_ = -1;
  UpdateAccessibility();
  updateGeometry();
  update();
  if (value_ != previous_value) {
    emit ValueChanged(value_);
  }
}

void Rating::SetClearEnabled(bool enabled) { clear_enabled_ = enabled; }

void Rating::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  update();
}

QSize Rating::sizeHint() const {
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  return QSize(maximum_ * tokens.control_height_small,
               tokens.control_height_medium);
}

QSize Rating::minimumSizeHint() const {
  return QSize(maximum_ * 20, GetThemeTokens(tone_).control_height_small);
}

void Rating::keyPressEvent(QKeyEvent* event) {
  switch (event->key()) {
    case Qt::Key_Left:
    case Qt::Key_Down:
      SetValue(value_ - 1);
      event->accept();
      return;
    case Qt::Key_Right:
    case Qt::Key_Up:
      SetValue(value_ + 1);
      event->accept();
      return;
    case Qt::Key_Home:
      SetValue(clear_enabled_ ? 0 : 1);
      event->accept();
      return;
    case Qt::Key_End:
      SetValue(maximum_);
      event->accept();
      return;
    default:
      break;
  }
  QAbstractButton::keyPressEvent(event);
}

void Rating::leaveEvent(QEvent* event) {
  hover_value_ = -1;
  update();
  QAbstractButton::leaveEvent(event);
}

void Rating::mouseMoveEvent(QMouseEvent* event) {
  const int hovered = ValueAtPosition(event->position().x());
  if (hover_value_ != hovered) {
    hover_value_ = hovered;
    update();
  }
  QAbstractButton::mouseMoveEvent(event);
}

void Rating::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    int selected = ValueAtPosition(event->position().x());
    if (clear_enabled_ && selected == value_) {
      selected = 0;
    }
    SetValue(selected);
    event->accept();
    return;
  }
  QAbstractButton::mousePressEvent(event);
}

void Rating::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  const int visible_value = hover_value_ >= 0 ? hover_value_ : value_;
  const qreal cell_width = static_cast<qreal>(width()) / maximum_;
  const qreal radius =
      qMax<qreal>(3.0, qMin(cell_width, static_cast<qreal>(height())) * 0.36);

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  for (int index = 0; index < maximum_; ++index) {
    const QPainterPath star =
        StarPath(QPointF((index + 0.5) * cell_width, height() / 2.0), radius);
    const bool filled = index < visible_value;
    painter.setPen(QPen(isEnabled() ? tokens.accent : tokens.disabled, 1.5));
    painter.setBrush(filled ? (isEnabled() ? tokens.accent : tokens.disabled)
                            : Qt::NoBrush);
    painter.drawPath(star);
  }
  if (hasFocus()) {
    painter.setPen(QPen(tokens.focus_ring, tokens.focus_ring_width));
    painter.setBrush(Qt::NoBrush);
    painter.drawRoundedRect(QRectF(rect()).adjusted(1.0, 1.0, -1.0, -1.0),
                            tokens.radius_small, tokens.radius_small);
  }
}

int Rating::ValueAtPosition(qreal x) const {
  if (width() <= 0) {
    return 0;
  }
  return qBound(1, static_cast<int>(x * maximum_ / width()) + 1, maximum_);
}

void Rating::UpdateAccessibility() {
  setAccessibleDescription(tr("%1 of %2 stars").arg(value_).arg(maximum_));
}

}  // namespace autoviz::widget
