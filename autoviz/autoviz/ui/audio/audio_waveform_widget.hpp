/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QWidget>

#include <cstdint>
#include <vector>

#include "autoviz/ui/audio/audio_timeline_buffer.hpp"

namespace autoviz {
namespace audio_panel {

class AudioWaveformWidget : public QWidget {
  Q_OBJECT

 public:
  explicit AudioWaveformWidget(QWidget* parent = nullptr);

  void setTimelineBuffer(const AudioTimelineBuffer* buffer);
  void setWaveformColor(const QColor& color);
  void setPlayheadTimestampNs(std::int64_t timestamp_ns);
  void setDefaultWindowSec(double window_sec);
  void resetView();

 signals:
  void seekRequested(std::int64_t timestamp_ns);

 protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

 private:
  void updateEnvelopeCache();
  std::int64_t timestampAtX(int x) const;

  const AudioTimelineBuffer* buffer_ = nullptr;
  QColor waveform_color_ = QColor(78, 152, 226);
  std::int64_t view_start_ns_ = 0;
  std::int64_t view_end_ns_ = 0;
  std::int64_t playhead_ns_ = 0;
  double default_window_sec_ = 30.0;
  std::vector<AudioEnvelopeColumn> envelope_;
  bool panning_ = false;
  int pan_anchor_x_ = 0;
  std::int64_t pan_anchor_start_ns_ = 0;
};

}  // namespace audio_panel
}  // namespace autoviz
