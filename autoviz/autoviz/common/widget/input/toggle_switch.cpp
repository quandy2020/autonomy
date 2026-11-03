/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/input/toggle_switch.hpp"

#include <QAbstractAnimation>
#include <QEasingCurve>
#include <QHideEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPropertyAnimation>
#include <QShowEvent>
#include <QStyle>
#include <QStyleOptionFocusRect>
#include <algorithm>

namespace autoviz::widget {

ToggleSwitch::ToggleSwitch(QWidget* parent)
    : QAbstractButton(parent),
      animation_(new QPropertyAnimation(this, "thumbPosition", this)) {
  setObjectName(QStringLiteral("AutovizToggleSwitch"));
  setProperty("autovizRole", QStringLiteral("toggleSwitch"));
  setCheckable(true);
  setCursor(Qt::PointingHandCursor);
  setFocusPolicy(Qt::StrongFocus);
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  setAccessibleName(tr("Toggle"));

  animation_->setEasingCurve(QEasingCurve::OutCubic);
  connect(this, &QAbstractButton::toggled, this,
          &ToggleSwitch::AnimateToCheckedState);
}

void ToggleSwitch::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  update();
}

QSize ToggleSwitch::sizeHint() const { return QSize(44, 24); }

QSize ToggleSwitch::minimumSizeHint() const { return sizeHint(); }

void ToggleSwitch::setThumbPosition(qreal position) {
  const qreal bounded = std::clamp(position, 0.0, 1.0);
  if (qFuzzyCompare(thumb_position_ + 1.0, bounded + 1.0)) {
    return;
  }
  thumb_position_ = bounded;
  update();
}

void ToggleSwitch::changeEvent(QEvent* event) {
  QAbstractButton::changeEvent(event);
  if (event->type() == QEvent::EnabledChange && !isEnabled()) {
    StopAnimationAtCurrentState();
  }
}

void ToggleSwitch::hideEvent(QHideEvent* event) {
  QAbstractButton::hideEvent(event);
  StopAnimationAtCurrentState();
}

bool ToggleSwitch::hitButton(const QPoint& position) const {
  return rect().contains(position);
}

void ToggleSwitch::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  const QRectF track = QRectF(rect()).adjusted(1.0, 1.0, -1.0, -1.0);
  const qreal radius = track.height() * 0.5;

  QColor track_color = isChecked() ? tokens.accent : tokens.border;
  if (!isEnabled()) {
    track_color = tokens.disabled;
  } else if (underMouse()) {
    track_color = isChecked() ? tokens.accent_hovered : tokens.surface_hovered;
  }

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(Qt::NoPen);
  painter.setBrush(track_color);
  painter.drawRoundedRect(track, radius, radius);

  constexpr qreal kInset = 3.0;
  const qreal diameter = track.height() - 2.0 * kInset;
  const qreal travel = track.width() - diameter - 2.0 * kInset;
  const qreal thumb_x = track.left() + kInset + travel * thumb_position_;
  const QRectF thumb(thumb_x, track.top() + kInset, diameter, diameter);
  painter.setBrush(isEnabled() ? tokens.surface_raised : tokens.surface);
  painter.drawEllipse(thumb);

  if (hasFocus()) {
    QStyleOptionFocusRect focus;
    focus.initFrom(this);
    focus.rect = rect().adjusted(0, 0, -1, -1);
    focus.backgroundColor = tokens.surface;
    style()->drawPrimitive(QStyle::PE_FrameFocusRect, &focus, &painter, this);
  }
}

void ToggleSwitch::showEvent(QShowEvent* event) {
  QAbstractButton::showEvent(event);
  setThumbPosition(isChecked() ? 1.0 : 0.0);
}

void ToggleSwitch::AnimateToCheckedState(bool checked) {
  const qreal target = checked ? 1.0 : 0.0;
  const bool animations_enabled =
      style()->styleHint(QStyle::SH_Widget_Animate, nullptr, this) != 0;
  if (!isVisible() || !isEnabled() || !animations_enabled) {
    animation_->stop();
    setThumbPosition(target);
    return;
  }

  animation_->stop();
  animation_->setDuration(GetThemeTokens(tone_).animation_fast_ms);
  animation_->setStartValue(thumb_position_);
  animation_->setEndValue(target);
  animation_->start(QAbstractAnimation::KeepWhenStopped);
}

void ToggleSwitch::StopAnimationAtCurrentState() {
  animation_->stop();
  setThumbPosition(isChecked() ? 1.0 : 0.0);
}

}  // namespace autoviz::widget
