/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/audio/audio_waveform_widget.hpp"

#include <algorithm>
#include <cmath>

#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

namespace autoviz {
namespace audio_panel {
namespace {

constexpr double kDefaultWindowSec = 30.0;
constexpr double kMinWindowSec = 0.25;
constexpr double kMaxWindowSec = 600.0;

}  // namespace

AudioWaveformWidget::AudioWaveformWidget(QWidget* parent) : QWidget(parent) {
  setMinimumHeight(120);
  setMouseTracking(true);
  setFocusPolicy(Qt::StrongFocus);
}

void AudioWaveformWidget::setTimelineBuffer(const AudioTimelineBuffer* buffer) {
  buffer_ = buffer;
  if (buffer_ == nullptr || buffer_->empty()) {
    envelope_.clear();
    update();
    return;
  }
  if (view_end_ns_ <= view_start_ns_) {
    view_end_ns_ = buffer_->endTimestampNs();
    view_start_ns_ = view_end_ns_ -
                     static_cast<std::int64_t>(default_window_sec_ * 1e9);
  }
  updateEnvelopeCache();
  update();
}

void AudioWaveformWidget::setWaveformColor(const QColor& color) {
  waveform_color_ = color;
  update();
}

void AudioWaveformWidget::setPlayheadTimestampNs(std::int64_t timestamp_ns) {
  playhead_ns_ = timestamp_ns;
  update();
}

void AudioWaveformWidget::setDefaultWindowSec(double window_sec) {
  default_window_sec_ =
      std::clamp(window_sec, kMinWindowSec, kMaxWindowSec);
}

void AudioWaveformWidget::resetView() {
  if (buffer_ == nullptr || buffer_->empty()) {
    view_start_ns_ = 0;
    view_end_ns_ = 0;
    envelope_.clear();
    update();
    return;
  }
  view_end_ns_ = buffer_->endTimestampNs();
  view_start_ns_ =
      view_end_ns_ - static_cast<std::int64_t>(default_window_sec_ * 1e9);
  updateEnvelopeCache();
  update();
}

void AudioWaveformWidget::updateEnvelopeCache() {
  envelope_.clear();
  if (buffer_ == nullptr || buffer_->empty() || width() <= 0 ||
      view_end_ns_ <= view_start_ns_) {
    return;
  }
  buffer_->buildEnvelope(view_start_ns_, view_end_ns_, width(), &envelope_);
}

std::int64_t AudioWaveformWidget::timestampAtX(int x) const {
  if (width() <= 0 || view_end_ns_ <= view_start_ns_) {
    return view_start_ns_;
  }
  const double normalized =
      std::clamp(static_cast<double>(x) / static_cast<double>(width()), 0.0, 1.0);
  return view_start_ns_ +
         static_cast<std::int64_t>(normalized * (view_end_ns_ - view_start_ns_));
}

void AudioWaveformWidget::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  QPainter painter(this);
  painter.fillRect(rect(), QColor(18, 18, 20));

  if (buffer_ == nullptr || buffer_->empty() || envelope_.empty()) {
    painter.setPen(QColor(140, 140, 140));
    painter.drawText(rect(), Qt::AlignCenter, tr("No audio data"));
    return;
  }

  const int mid_y = height() / 2;
  const double half_height = std::max(height() * 0.42, 10.0);
  painter.setPen(QPen(waveform_color_, 1.0));
  int last_x = 0;
  bool has_point = false;
  for (int x = 0; x < static_cast<int>(envelope_.size()); ++x) {
    const AudioEnvelopeColumn& column = envelope_[static_cast<std::size_t>(x)];
    if (column.max_amplitude == 0.0f && column.min_amplitude == 0.0f) {
      continue;
    }
    const int y_min = mid_y - static_cast<int>(column.max_amplitude * half_height);
    const int y_max = mid_y - static_cast<int>(column.min_amplitude * half_height);
    painter.drawLine(x, y_min, x, y_max);
    if (has_point && x - last_x > 1) {
      painter.drawLine(last_x, mid_y, x, mid_y);
    }
    last_x = x;
    has_point = true;
  }

  if (playhead_ns_ >= view_start_ns_ && playhead_ns_ <= view_end_ns_) {
    const double normalized =
        static_cast<double>(playhead_ns_ - view_start_ns_) /
        static_cast<double>(view_end_ns_ - view_start_ns_);
    const int x = static_cast<int>(normalized * width());
    painter.setPen(QPen(QColor(255, 180, 60), 1.5));
    painter.drawLine(x, 0, x, height());
  }
}

void AudioWaveformWidget::mousePressEvent(QMouseEvent* event) {
  if (event == nullptr || buffer_ == nullptr || buffer_->empty()) {
    return;
  }
  if (event->button() == Qt::LeftButton) {
    if (event->modifiers() & Qt::ShiftModifier) {
      panning_ = true;
      pan_anchor_x_ = event->position().x();
      pan_anchor_start_ns_ = view_start_ns_;
      return;
    }
    const std::int64_t timestamp_ns = timestampAtX(static_cast<int>(event->position().x()));
    playhead_ns_ = timestamp_ns;
    emit seekRequested(timestamp_ns);
    update();
  }
}

void AudioWaveformWidget::mouseMoveEvent(QMouseEvent* event) {
  if (event == nullptr || !panning_) {
    return;
  }
  const int delta_x = static_cast<int>(event->position().x()) - pan_anchor_x_;
  const std::int64_t span_ns = view_end_ns_ - view_start_ns_;
  if (width() <= 0) {
    return;
  }
  const std::int64_t delta_ns =
      static_cast<std::int64_t>(-static_cast<double>(delta_x) / width() * span_ns);
  view_start_ns_ = pan_anchor_start_ns_ + delta_ns;
  view_end_ns_ = view_start_ns_ + span_ns;
  updateEnvelopeCache();
  update();
}

void AudioWaveformWidget::mouseReleaseEvent(QMouseEvent* event) {
  Q_UNUSED(event);
  panning_ = false;
}

void AudioWaveformWidget::wheelEvent(QWheelEvent* event) {
  if (event == nullptr || buffer_ == nullptr || buffer_->empty() ||
      view_end_ns_ <= view_start_ns_) {
    return;
  }
  const double factor = event->angleDelta().y() > 0 ? 0.85 : 1.0 / 0.85;
  const std::int64_t anchor_ns = timestampAtX(static_cast<int>(event->position().x()));
  std::int64_t span_ns = view_end_ns_ - view_start_ns_;
  span_ns = static_cast<std::int64_t>(
      std::clamp(static_cast<double>(span_ns) * factor,
                 kMinWindowSec * 1e9, kMaxWindowSec * 1e9));
  const double anchor_ratio =
      static_cast<double>(anchor_ns - view_start_ns_) /
      static_cast<double>(view_end_ns_ - view_start_ns_);
  view_start_ns_ = anchor_ns - static_cast<std::int64_t>(anchor_ratio * span_ns);
  view_end_ns_ = view_start_ns_ + span_ns;
  updateEnvelopeCache();
  update();
  event->accept();
}

void AudioWaveformWidget::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  updateEnvelopeCache();
}

}  // namespace audio_panel
}  // namespace autoviz
