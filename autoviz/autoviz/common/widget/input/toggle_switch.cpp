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
#include <algorithm>

#include "autoviz/common/widget/foundation/motion_policy.hpp"

namespace autoviz::widget {
namespace {

constexpr qreal kTrackHeight = 31.0;
constexpr qreal kThumbDiameter = 27.0;
constexpr qreal kThumbInset = 2.0;

QColor WithOpacity(QColor color, qreal opacity) {
  color.setAlpha(qRound(color.alpha() * std::clamp(opacity, 0.0, 1.0)));
  return color;
}

}  // namespace

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

QSize ToggleSwitch::sizeHint() const { return QSize(51, 44); }

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
  if (event->type() == QEvent::EnabledChange) {
    setCursor(isEnabled() ? Qt::PointingHandCursor : Qt::ArrowCursor);
    if (!isEnabled()) {
      StopAnimationAtCurrentState();
    }
    update();
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
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  const QRectF track(0.5, (height() - kTrackHeight) * 0.5,
                     qMax(0.0, width() - 1.0), kTrackHeight);
  const qreal radius = track.height() * 0.5;

  QColor track_color = isChecked() ? tokens.accent : tokens.surface_sunken;
  if (!isEnabled()) {
    track_color = WithOpacity(track_color, isChecked() ? 0.42 : 0.58);
  } else if (isDown()) {
    track_color = isChecked() ? tokens.accent_pressed : tokens.surface_pressed;
  } else if (underMouse()) {
    track_color = isChecked() ? tokens.accent_hovered : tokens.surface_material;
  }

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  if (isChecked()) {
    painter.setPen(Qt::NoPen);
  } else {
    painter.setPen(QPen(WithOpacity(tokens.border_subtle, 0.72), 1.0));
  }
  painter.setBrush(track_color);
  painter.drawRoundedRect(track, radius, radius);

  const qreal travel = track.width() - kThumbDiameter - 2.0 * kThumbInset;
  const qreal logical_position = layoutDirection() == Qt::RightToLeft
                                     ? 1.0 - thumb_position_
                                     : thumb_position_;
  const qreal thumb_x = track.left() + kThumbInset + travel * logical_position;
  const QRectF thumb(thumb_x, track.top() + kThumbInset, kThumbDiameter,
                     kThumbDiameter);
  painter.setPen(Qt::NoPen);
  painter.setBrush(WithOpacity(QColor(Qt::black), tokens.is_dark ? 0.34 : 0.2));
  painter.drawEllipse(thumb.translated(0.0, 2.0));
  painter.setBrush(WithOpacity(QColor(Qt::black), tokens.is_dark ? 0.2 : 0.12));
  painter.drawEllipse(thumb.translated(0.0, 1.0));
  painter.setBrush(WithOpacity(QColor(Qt::white), isEnabled() ? 1.0 : 0.74));
  painter.setPen(QPen(WithOpacity(QColor(Qt::black), 0.08), 0.75));
  painter.drawEllipse(thumb);

  if (hasFocus()) {
    painter.setBrush(Qt::NoBrush);
    painter.setPen(
        QPen(tokens.focus_ring_outer, tokens.focus_ring_width + 2.0));
    painter.drawRoundedRect(track.adjusted(-3.0, -3.0, 3.0, 3.0), radius + 3.0,
                            radius + 3.0);
    painter.setPen(QPen(tokens.focus_ring, 1.5));
    painter.drawRoundedRect(track.adjusted(-1.0, -1.0, 1.0, 1.0), radius + 1.0,
                            radius + 1.0);
  }
}

void ToggleSwitch::showEvent(QShowEvent* event) {
  QAbstractButton::showEvent(event);
  setThumbPosition(isChecked() ? 1.0 : 0.0);
}

void ToggleSwitch::AnimateToCheckedState(bool checked) {
  const qreal target = checked ? 1.0 : 0.0;
  const int duration = MotionPolicy::ResolveDuration(
      GetThemeTokens(this, tone_).animation_normal_ms);
  if (!isVisible() || !isEnabled() || duration == 0) {
    animation_->stop();
    setThumbPosition(target);
    return;
  }

  animation_->stop();
  animation_->setDuration(duration);
  animation_->setStartValue(thumb_position_);
  animation_->setEndValue(target);
  animation_->start(QAbstractAnimation::KeepWhenStopped);
}

void ToggleSwitch::StopAnimationAtCurrentState() {
  animation_->stop();
  setThumbPosition(isChecked() ? 1.0 : 0.0);
}

}  // namespace autoviz::widget
