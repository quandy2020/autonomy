/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/feedback/progress_ring.hpp"

#include <QEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QShowEvent>
#include <QStyle>
#include <QTimerEvent>
#include <algorithm>

namespace autoviz::widget {

ProgressRing::ProgressRing(QWidget* parent) : QWidget(parent) {
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  setAccessibleName(tr("Progress"));
  UpdateAccessibleValue();
}

void ProgressRing::SetRange(int minimum, int maximum) {
  if (minimum > maximum) std::swap(minimum, maximum);
  if (minimum_ == minimum && maximum_ == maximum) return;
  minimum_ = minimum;
  maximum_ = maximum;
  value_ = qBound(minimum_, value_, maximum_);
  UpdateAccessibleValue();
  update();
}

void ProgressRing::SetValue(int value) {
  const int bounded = qBound(minimum_, value, maximum_);
  if (value_ == bounded) return;
  value_ = bounded;
  UpdateAccessibleValue();
  update();
}

void ProgressRing::SetIndeterminate(bool indeterminate) {
  if (indeterminate_ == indeterminate) return;
  indeterminate_ = indeterminate;
  SyncAnimation();
  UpdateAccessibleValue();
  update();
}

void ProgressRing::SetStrokeWidth(int width) {
  const int bounded = qBound(1, width, 12);
  if (stroke_width_ == bounded) return;
  stroke_width_ = bounded;
  update();
}

void ProgressRing::SetTone(style::Tone tone) {
  if (tone_ == tone) return;
  tone_ = tone;
  update();
}

QSize ProgressRing::sizeHint() const { return QSize(32, 32); }

QSize ProgressRing::minimumSizeHint() const { return QSize(16, 16); }

void ProgressRing::changeEvent(QEvent* event) {
  QWidget::changeEvent(event);
  if (event->type() == QEvent::EnabledChange ||
      event->type() == QEvent::StyleChange) {
    SyncAnimation();
  }
}

void ProgressRing::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  const qreal inset = stroke_width_ * 0.5 + 1.0;
  const QRectF arc_rect = QRectF(rect()).adjusted(inset, inset, -inset, -inset);
  const qint64 range =
      static_cast<qint64>(maximum_) - static_cast<qint64>(minimum_);
  const qint64 progress =
      static_cast<qint64>(value_) - static_cast<qint64>(minimum_);
  const qreal fraction =
      range > 0 ? static_cast<qreal>(progress) / static_cast<qreal>(range)
                : 0.0;

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setBrush(Qt::NoBrush);
  painter.setPen(
      QPen(tokens.border, stroke_width_, Qt::SolidLine, Qt::RoundCap));
  painter.drawEllipse(arc_rect);
  painter.setPen(QPen(isEnabled() ? tokens.accent : tokens.disabled,
                      stroke_width_, Qt::SolidLine, Qt::RoundCap));
  const int start_angle = (90 - rotation_degrees_) * 16;
  const int span_angle =
      indeterminate_ ? -110 * 16 : qRound(-360.0 * fraction * 16.0);
  painter.drawArc(arc_rect, start_angle, span_angle);
}

void ProgressRing::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  SyncAnimation();
}

void ProgressRing::hideEvent(QHideEvent* event) {
  QWidget::hideEvent(event);
  SyncAnimation();
}

void ProgressRing::timerEvent(QTimerEvent* event) {
  if (event->timerId() != timer_.timerId()) {
    QWidget::timerEvent(event);
    return;
  }
  rotation_degrees_ = (rotation_degrees_ + 8) % 360;
  update();
}

void ProgressRing::SyncAnimation() {
  const bool animations_enabled =
      style()->styleHint(QStyle::SH_Widget_Animate, nullptr, this) != 0;
  const bool should_run =
      indeterminate_ && isVisible() && isEnabled() && animations_enabled;
  if (should_run && !timer_.isActive()) {
    timer_.start(16, Qt::PreciseTimer, this);
  } else if (!should_run && timer_.isActive()) {
    timer_.stop();
  }
}

void ProgressRing::UpdateAccessibleValue() {
  if (indeterminate_) {
    setAccessibleDescription(tr("Progress is indeterminate"));
    return;
  }

  const qint64 range =
      static_cast<qint64>(maximum_) - static_cast<qint64>(minimum_);
  const qint64 progress =
      static_cast<qint64>(value_) - static_cast<qint64>(minimum_);
  const int percent =
      range > 0
          ? qRound(100.0 * static_cast<qreal>(progress) /
                   static_cast<qreal>(range))
          : 0;
  setAccessibleDescription(
      tr("%1 percent; value %2; range %3 to %4")
          .arg(percent)
          .arg(value_)
          .arg(minimum_)
          .arg(maximum_));
}

}  // namespace autoviz::widget
