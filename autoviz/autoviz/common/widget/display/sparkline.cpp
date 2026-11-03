/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/display/sparkline.hpp"

#include <QPaintEvent>
#include <QPainter>
#include <QResizeEvent>
#include <QVariant>
#include <algorithm>
#include <cmath>
#include <limits>

namespace autoviz::widget {
namespace {

constexpr int kRefreshIntervalMs = 16;

}  // namespace

Sparkline::Sparkline(QWidget* parent) : QWidget(parent), refresh_timer_(this) {
  setObjectName(QStringLiteral("AutovizSparkline"));
  setProperty("autovizRole", QStringLiteral("sparkline"));
  setAttribute(Qt::WA_TransparentForMouseEvents);
  setAccessibleName(tr("Metric trend"));
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  samples_.reserve(capacity_);
  line_points_.reserve(capacity_);
  fill_points_.reserve(capacity_ + 2);
  refresh_timer_.setInterval(kRefreshIntervalMs);
  refresh_timer_.setSingleShot(true);
  connect(&refresh_timer_, &QTimer::timeout, this,
          [this]() { FlushPendingRefresh(); });
}

void Sparkline::SetCapacity(int capacity) {
  const int bounded = qBound(2, capacity, 100000);
  if (capacity_ == bounded) {
    return;
  }

  QVector<double> retained;
  const int sample_count = static_cast<int>(samples_.size());
  const int keep = qMin(bounded, sample_count);
  retained.reserve(bounded);
  for (int index = sample_count - keep; index < sample_count; ++index) {
    retained.push_back(SampleAt(index));
  }
  capacity_ = bounded;
  samples_ = std::move(retained);
  samples_.reserve(capacity_);
  head_ = 0;
  RebuildRangeIndex();
  ApplyAutoRange();
  RebuildGeometry();
  UpdateAccessibleValue();
  update();
}

void Sparkline::AppendSample(double value) {
  if (!std::isfinite(value)) {
    return;
  }
  AppendFiniteSample(value);
  ApplyAutoRange();
  ScheduleRefresh();
}

void Sparkline::AppendSamples(const QVector<double>& values) {
  if (values.isEmpty()) {
    return;
  }
  for (double value : values) {
    if (!std::isfinite(value)) {
      continue;
    }
    AppendFiniteSample(value);
  }
  ApplyAutoRange();
  ScheduleRefresh();
}

void Sparkline::Clear() {
  if (samples_.isEmpty()) {
    return;
  }
  samples_.clear();
  line_points_.clear();
  fill_points_.clear();
  minimum_queue_.clear();
  maximum_queue_.clear();
  refresh_timer_.stop();
  head_ = 0;
  next_sequence_ = 0;
  refresh_pending_ = false;
  if (auto_range_) {
    minimum_ = 0.0;
    maximum_ = 1.0;
  }
  UpdateAccessibleValue();
  update();
}

void Sparkline::SetAutoRange(bool enabled) {
  if (auto_range_ == enabled) {
    return;
  }
  auto_range_ = enabled;
  if (auto_range_) {
    ApplyAutoRange();
    RebuildGeometry();
    update();
  }
}

void Sparkline::SetRange(double minimum, double maximum) {
  if (!std::isfinite(minimum) || !std::isfinite(maximum)) {
    return;
  }
  if (minimum > maximum) {
    std::swap(minimum, maximum);
  }
  if (qFuzzyCompare(minimum_ + 1.0, minimum + 1.0) &&
      qFuzzyCompare(maximum_ + 1.0, maximum + 1.0) && !auto_range_) {
    return;
  }
  if (qFuzzyCompare(minimum, maximum)) {
    const double padding = qMax(1.0, std::abs(minimum) * 0.05);
    minimum -= padding;
    maximum += padding;
  }
  minimum_ = minimum;
  maximum_ = maximum;
  auto_range_ = false;
  RebuildGeometry();
  update();
}

void Sparkline::SetLineColor(const QColor& color) {
  if (line_color_ == color) {
    return;
  }
  line_color_ = color;
  update();
}

void Sparkline::SetFillEnabled(bool enabled) {
  if (fill_enabled_ == enabled) {
    return;
  }
  fill_enabled_ = enabled;
  update();
}

void Sparkline::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  update();
}

QSize Sparkline::sizeHint() const { return QSize(180, 56); }

QSize Sparkline::minimumSizeHint() const { return QSize(64, 32); }

void Sparkline::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  const QColor line =
      isEnabled() ? (line_color_.isValid() ? line_color_ : tokens.accent)
                  : tokens.disabled;

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(QPen(tokens.border, 1));
  painter.drawLine(0, height() - 1, width(), height() - 1);
  if (line_points_.size() < 2) {
    return;
  }
  if (fill_enabled_ && fill_points_.size() >= 4) {
    QColor fill = line;
    fill.setAlpha(42);
    painter.setPen(Qt::NoPen);
    painter.setBrush(fill);
    painter.drawPolygon(fill_points_);
  }
  painter.setBrush(Qt::NoBrush);
  painter.setPen(QPen(line, 1.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  painter.drawPolyline(line_points_);
}

void Sparkline::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  RebuildGeometry();
}

void Sparkline::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  if (refresh_pending_) {
    FlushPendingRefresh();
  }
}

void Sparkline::hideEvent(QHideEvent* event) {
  if (refresh_timer_.isActive()) {
    refresh_timer_.stop();
    refresh_pending_ = true;
  }
  QWidget::hideEvent(event);
}

void Sparkline::AppendFiniteSample(double value) {
  if (next_sequence_ == std::numeric_limits<quint64>::max()) {
    RebuildRangeIndex();
  }
  const quint64 sequence = next_sequence_++;
  const bool at_capacity = static_cast<int>(samples_.size()) == capacity_;
  const quint64 evicted_sequence =
      at_capacity ? sequence - static_cast<quint64>(samples_.size()) : 0;

  if (!at_capacity) {
    samples_.push_back(value);
  } else {
    samples_[head_] = value;
    head_ = (head_ + 1) % capacity_;
    while (!minimum_queue_.empty() &&
           minimum_queue_.front().sequence <= evicted_sequence) {
      minimum_queue_.pop_front();
    }
    while (!maximum_queue_.empty() &&
           maximum_queue_.front().sequence <= evicted_sequence) {
      maximum_queue_.pop_front();
    }
  }

  while (!minimum_queue_.empty() && minimum_queue_.back().value >= value) {
    minimum_queue_.pop_back();
  }
  minimum_queue_.push_back({sequence, value});
  while (!maximum_queue_.empty() && maximum_queue_.back().value <= value) {
    maximum_queue_.pop_back();
  }
  maximum_queue_.push_back({sequence, value});
}

void Sparkline::ApplyAutoRange() {
  if (!auto_range_) {
    return;
  }
  if (minimum_queue_.empty() || maximum_queue_.empty()) {
    minimum_ = 0.0;
    maximum_ = 1.0;
    return;
  }
  minimum_ = minimum_queue_.front().value;
  maximum_ = maximum_queue_.front().value;
  if (qFuzzyCompare(minimum_, maximum_)) {
    const double padding = qMax(1.0, std::abs(minimum_) * 0.05);
    minimum_ -= padding;
    maximum_ += padding;
  }
}

void Sparkline::RebuildRangeIndex() {
  minimum_queue_.clear();
  maximum_queue_.clear();
  next_sequence_ = 0;
  const int count = static_cast<int>(samples_.size());
  for (int index = 0; index < count; ++index) {
    const double value = SampleAt(index);
    const quint64 sequence = next_sequence_++;
    while (!minimum_queue_.empty() && minimum_queue_.back().value >= value) {
      minimum_queue_.pop_back();
    }
    minimum_queue_.push_back({sequence, value});
    while (!maximum_queue_.empty() && maximum_queue_.back().value <= value) {
      maximum_queue_.pop_back();
    }
    maximum_queue_.push_back({sequence, value});
  }
}

void Sparkline::ScheduleRefresh() {
  refresh_pending_ = true;
  if (isVisible() && !refresh_timer_.isActive()) {
    refresh_timer_.start();
  }
}

void Sparkline::FlushPendingRefresh() {
  if (!refresh_pending_) {
    return;
  }
  refresh_pending_ = false;
  RebuildGeometry();
  UpdateAccessibleValue();
  update();
}

double Sparkline::SampleAt(int logical_index) const {
  if (static_cast<int>(samples_.size()) < capacity_) {
    return samples_[logical_index];
  }
  return samples_[(head_ + logical_index) % capacity_];
}

void Sparkline::RebuildGeometry() {
  const int count = static_cast<int>(samples_.size());
  line_points_.clear();
  fill_points_.clear();
  if (count == 0) {
    return;
  }
  const qreal drawable_width = qMax(0, width() - 1);
  const qreal drawable_height = qMax(0, height() - 2);
  const double range = maximum_ - minimum_;
  const auto append_point = [this, drawable_width, drawable_height, range,
                             count](int index, double value) {
    const qreal x = count > 1
                        ? drawable_width * index / static_cast<qreal>(count - 1)
                        : drawable_width;
    const double normalized = range > 0.0 ? (value - minimum_) / range : 0.5;
    const qreal y =
        drawable_height * (1.0 - qBound<qreal>(0.0, normalized, 1.0));
    line_points_.push_back(QPointF(x, y));
  };

  const int column_count = qMax(1, qMin(count, qMax(1, width())));
  if (count <= column_count * 2) {
    line_points_.reserve(count);
    for (int index = 0; index < count; ++index) {
      append_point(index, SampleAt(index));
    }
  } else {
    line_points_.reserve(column_count * 2);
    for (int column = 0; column < column_count; ++column) {
      const int begin =
          static_cast<int>(static_cast<qint64>(column) * count / column_count);
      const int end = static_cast<int>(static_cast<qint64>(column + 1) * count /
                                       column_count);
      int minimum_index = begin;
      int maximum_index = begin;
      double bucket_minimum = SampleAt(begin);
      double bucket_maximum = bucket_minimum;
      for (int index = begin + 1; index < end; ++index) {
        const double value = SampleAt(index);
        if (value < bucket_minimum) {
          bucket_minimum = value;
          minimum_index = index;
        }
        if (value > bucket_maximum) {
          bucket_maximum = value;
          maximum_index = index;
        }
      }
      if (minimum_index <= maximum_index) {
        append_point(minimum_index, bucket_minimum);
        if (maximum_index != minimum_index) {
          append_point(maximum_index, bucket_maximum);
        }
      } else {
        append_point(maximum_index, bucket_maximum);
        append_point(minimum_index, bucket_minimum);
      }
    }
  }
  const int point_count = static_cast<int>(line_points_.size());
  if (point_count >= 2) {
    fill_points_.resize(point_count + 2);
    fill_points_[0] = QPointF(line_points_.front().x(), drawable_height);
    for (int index = 0; index < point_count; ++index) {
      fill_points_[index + 1] = line_points_[index];
    }
    fill_points_[point_count + 1] =
        QPointF(line_points_.back().x(), drawable_height);
  }
}

void Sparkline::UpdateAccessibleValue() {
  if (samples_.isEmpty()) {
    setAccessibleDescription(tr("No samples"));
    return;
  }
  setAccessibleDescription(
      tr("%1 samples, latest value %2")
          .arg(samples_.size())
          .arg(SampleAt(static_cast<int>(samples_.size()) - 1), 0, 'g', 8));
}

}  // namespace autoviz::widget
