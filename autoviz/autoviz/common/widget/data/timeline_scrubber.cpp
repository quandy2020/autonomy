/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/data/timeline_scrubber.hpp"

#include <QKeyEvent>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPointer>
#include <QResizeEvent>
#include <QVariant>
#include <algorithm>
#include <cmath>
#include <limits>

namespace autoviz::widget {
namespace {

qint64 BoundedAdd(qint64 value, qint64 delta, qint64 minimum, qint64 maximum) {
  const long double candidate =
      static_cast<long double>(value) + static_cast<long double>(delta);
  if (candidate <= static_cast<long double>(minimum)) {
    return minimum;
  }
  if (candidate >= static_cast<long double>(maximum)) {
    return maximum;
  }
  return static_cast<qint64>(candidate);
}

qint64 RangeStep(qint64 minimum, qint64 maximum, int divisor) {
  const long double span =
      static_cast<long double>(maximum) - static_cast<long double>(minimum);
  const long double step = span / divisor;
  if (step >= static_cast<long double>(std::numeric_limits<qint64>::max())) {
    return std::numeric_limits<qint64>::max();
  }
  return qMax<qint64>(1, static_cast<qint64>(step));
}

quint64 UnsignedDistance(qint64 lower, qint64 upper) {
  return static_cast<quint64>(upper) - static_cast<quint64>(lower);
}

}  // namespace

TimelineScrubber::TimelineScrubber(QWidget* parent) : QWidget(parent) {
  setObjectName(QStringLiteral("AutovizTimelineScrubber"));
  setProperty("autovizRole", QStringLiteral("timelineScrubber"));
  setFocusPolicy(Qt::StrongFocus);
  setCursor(Qt::PointingHandCursor);
  setAccessibleName(tr("Timeline"));
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  UpdateAccessibleValue();
}

void TimelineScrubber::SetRange(qint64 minimum, qint64 maximum) {
  if (minimum > maximum) {
    std::swap(minimum, maximum);
  }
  if (minimum_ == minimum && maximum_ == maximum) {
    return;
  }
  minimum_ = minimum;
  maximum_ = maximum;
  const qint64 bounded = qBound(minimum_, value_, maximum_);
  const bool value_changed = bounded != value_;
  value_ = bounded;
  RebuildMarkerPositions();
  UpdateAccessibleValue();
  update();
  if (value_changed) {
    emit ValueChanged(value_);
  }
}

void TimelineScrubber::SetValue(qint64 value) {
  const qint64 bounded = qBound(minimum_, value, maximum_);
  if (value_ == bounded) {
    return;
  }
  value_ = bounded;
  UpdateAccessibleValue();
  update();
  emit ValueChanged(value_);
}

void TimelineScrubber::SetMarkers(QVector<TimelineMarker> markers) {
  std::sort(markers.begin(), markers.end(),
            [](const TimelineMarker& lhs, const TimelineMarker& rhs) {
              return lhs.position < rhs.position;
            });
  markers_ = std::move(markers);
  RebuildMarkerPositions();
  update();
}

void TimelineScrubber::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  update();
}

QSize TimelineScrubber::sizeHint() const { return QSize(360, 44); }

QSize TimelineScrubber::minimumSizeHint() const { return QSize(120, 36); }

void TimelineScrubber::keyPressEvent(QKeyEvent* event) {
  const qint64 fine_step = RangeStep(minimum_, maximum_, 100);
  const qint64 coarse_step = RangeStep(minimum_, maximum_, 10);
  switch (event->key()) {
    case Qt::Key_Left:
      SetValue(BoundedAdd(value_, -fine_step, minimum_, maximum_));
      break;
    case Qt::Key_Right:
      SetValue(BoundedAdd(value_, fine_step, minimum_, maximum_));
      break;
    case Qt::Key_PageDown:
      SetValue(BoundedAdd(value_, -coarse_step, minimum_, maximum_));
      break;
    case Qt::Key_PageUp:
      SetValue(BoundedAdd(value_, coarse_step, minimum_, maximum_));
      break;
    case Qt::Key_Home:
      SetValue(minimum_);
      break;
    case Qt::Key_End:
      SetValue(maximum_);
      break;
    default:
      QWidget::keyPressEvent(event);
      return;
  }
  event->accept();
}

void TimelineScrubber::mouseMoveEvent(QMouseEvent* event) {
  if (dragging_) {
    SetValue(PositionFromX(event->position().x()));
    event->accept();
    return;
  }
  QWidget::mouseMoveEvent(event);
}

void TimelineScrubber::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    dragging_ = true;
    setFocus(Qt::MouseFocusReason);
    const qint64 position = PositionFromX(event->position().x());
    event->accept();
    QPointer<TimelineScrubber> guard(this);
    emit ScrubStarted();
    if (guard != nullptr) {
      guard->SetValue(position);
    }
    return;
  }
  QWidget::mousePressEvent(event);
}

void TimelineScrubber::mouseReleaseEvent(QMouseEvent* event) {
  if (dragging_ && event->button() == Qt::LeftButton) {
    dragging_ = false;
    const qreal release_x = event->position().x();
    const qint64 position = PositionFromX(release_x);
    const int marker = HitMarker(release_x);
    event->accept();
    QPointer<TimelineScrubber> guard(this);
    SetValue(position);
    if (guard == nullptr) {
      return;
    }
    if (marker >= 0) {
      emit MarkerActivated(marker);
      if (guard == nullptr) {
        return;
      }
    }
    emit ScrubFinished(guard->value());
    return;
  }
  QWidget::mouseReleaseEvent(event);
}

void TimelineScrubber::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  const QRectF track = TrackRect();
  const qreal value_x = XFromPosition(value_);

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(QPen(tokens.border, 4, Qt::SolidLine, Qt::RoundCap));
  painter.drawLine(QPointF(track.left(), track.center().y()),
                   QPointF(track.right(), track.center().y()));
  painter.setPen(QPen(isEnabled() ? tokens.accent : tokens.disabled, 4,
                      Qt::SolidLine, Qt::RoundCap));
  painter.drawLine(QPointF(track.left(), track.center().y()),
                   QPointF(value_x, track.center().y()));

  for (int index = 0; index < static_cast<int>(markers_.size()); ++index) {
    if (markers_[index].position < minimum_ ||
        markers_[index].position > maximum_) {
      continue;
    }
    const QColor color = markers_[index].color.isValid() ? markers_[index].color
                                                         : tokens.warning;
    painter.setPen(QPen(color, 2));
    painter.drawLine(QPointF(marker_positions_[index], track.top() - 5),
                     QPointF(marker_positions_[index], track.bottom() + 5));
  }

  painter.setPen(QPen(tokens.surface_raised, 2));
  painter.setBrush(isEnabled() ? tokens.accent : tokens.disabled);
  painter.drawEllipse(QPointF(value_x, track.center().y()), 7, 7);
  if (hasFocus()) {
    painter.setBrush(Qt::NoBrush);
    painter.setPen(QPen(tokens.focus_ring, tokens.focus_ring_width));
    painter.drawEllipse(QPointF(value_x, track.center().y()), 10, 10);
  }
}

void TimelineScrubber::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  RebuildMarkerPositions();
}

QRectF TimelineScrubber::TrackRect() const {
  const int margin = GetThemeTokens(tone_).spacing_lg;
  return QRectF(rect()).adjusted(margin, 12, -margin, -12);
}

qint64 TimelineScrubber::PositionFromX(qreal x) const {
  const QRectF track = TrackRect();
  if (maximum_ <= minimum_ || track.width() <= 0.0) {
    return minimum_;
  }
  const qreal fraction =
      qBound<qreal>(0.0, (x - track.left()) / track.width(), 1.0);
  if (fraction <= 0.0) {
    return minimum_;
  }
  if (fraction >= 1.0) {
    return maximum_;
  }
  const long double candidate =
      static_cast<long double>(minimum_) +
      static_cast<long double>(fraction) * (static_cast<long double>(maximum_) -
                                            static_cast<long double>(minimum_));
  if (candidate <= static_cast<long double>(minimum_)) {
    return minimum_;
  }
  if (candidate >= static_cast<long double>(maximum_)) {
    return maximum_;
  }
  return static_cast<qint64>(std::round(candidate));
}

qreal TimelineScrubber::XFromPosition(qint64 position) const {
  const QRectF track = TrackRect();
  if (maximum_ <= minimum_) {
    return track.left();
  }
  const qreal fraction =
      (static_cast<qreal>(position) - static_cast<qreal>(minimum_)) /
      (static_cast<qreal>(maximum_) - static_cast<qreal>(minimum_));
  return track.left() + qBound<qreal>(0.0, fraction, 1.0) * track.width();
}

void TimelineScrubber::RebuildMarkerPositions() {
  marker_positions_.clear();
  marker_positions_.reserve(markers_.size());
  for (const TimelineMarker& marker : markers_) {
    marker_positions_.push_back(XFromPosition(marker.position));
  }
}

int TimelineScrubber::HitMarker(qreal x) const {
  constexpr qreal kHitDistance = 5.0;
  for (int index = 0; index < static_cast<int>(marker_positions_.size());
       ++index) {
    if (markers_[index].position < minimum_ ||
        markers_[index].position > maximum_) {
      continue;
    }
    if (std::abs(marker_positions_[index] - x) <= kHitDistance) {
      return index;
    }
  }
  return -1;
}

void TimelineScrubber::UpdateAccessibleValue() {
  const QString position = QString::number(UnsignedDistance(minimum_, value_));
  const QString span = QString::number(UnsignedDistance(minimum_, maximum_));
  setAccessibleDescription(tr("Position %1 of %2").arg(position, span));
}

}  // namespace autoviz::widget
