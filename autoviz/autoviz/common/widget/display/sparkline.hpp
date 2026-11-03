/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QHideEvent>
#include <QPolygonF>
#include <QShowEvent>
#include <QTimer>
#include <QVector>
#include <QWidget>
#include <deque>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QPaintEvent;
class QResizeEvent;

namespace autoviz::widget {

// Lightweight fixed-capacity metric graph. Geometry is cached when samples,
// bounds, or size change; paintEvent performs no container allocation.
class Sparkline final : public QWidget {
 public:
  explicit Sparkline(QWidget* parent = nullptr);

  int capacity() const noexcept { return capacity_; }
  void SetCapacity(int capacity);
  int sample_count() const noexcept {
    return static_cast<int>(samples_.size());
  }
  void AppendSample(double value);
  void AppendSamples(const QVector<double>& values);
  void Clear();

  bool auto_range() const noexcept { return auto_range_; }
  void SetAutoRange(bool enabled);
  void SetRange(double minimum, double maximum);
  void SetLineColor(const QColor& color);
  void SetFillEnabled(bool enabled);
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 protected:
  void paintEvent(QPaintEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;
  void showEvent(QShowEvent* event) override;
  void hideEvent(QHideEvent* event) override;

 private:
  struct RangeSample {
    quint64 sequence = 0;
    double value = 0.0;
  };

  void AppendFiniteSample(double value);
  void ApplyAutoRange();
  void RebuildRangeIndex();
  void ScheduleRefresh();
  void FlushPendingRefresh();
  double SampleAt(int logical_index) const;
  void RebuildGeometry();
  void UpdateAccessibleValue();

  QVector<double> samples_;
  QPolygonF line_points_;
  QPolygonF fill_points_;
  QColor line_color_;
  QTimer refresh_timer_;
  std::deque<RangeSample> minimum_queue_;
  std::deque<RangeSample> maximum_queue_;
  style::Tone tone_ = style::Tone::Shell;
  int capacity_ = 240;
  int head_ = 0;
  quint64 next_sequence_ = 0;
  double minimum_ = 0.0;
  double maximum_ = 1.0;
  bool auto_range_ = true;
  bool fill_enabled_ = false;
  bool refresh_pending_ = false;
};

}  // namespace autoviz::widget
