/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QVector>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QKeyEvent;
class QMouseEvent;
class QPaintEvent;
class QResizeEvent;

namespace autoviz::widget {

struct TimelineMarker {
  qint64 position = 0;
  QColor color;
  QString description;
};

// Allocation-free paint-path scrubber for rosbag time, diagnostics, and event
// markers. Position units are selected by the caller (for example nanoseconds).
class TimelineScrubber final : public QWidget {
  Q_OBJECT

 public:
  explicit TimelineScrubber(QWidget* parent = nullptr);

  qint64 minimum() const noexcept { return minimum_; }
  qint64 maximum() const noexcept { return maximum_; }
  qint64 value() const noexcept { return value_; }
  void SetRange(qint64 minimum, qint64 maximum);
  void SetValue(qint64 value);
  void SetMarkers(QVector<TimelineMarker> markers);
  const QVector<TimelineMarker>& markers() const noexcept { return markers_; }
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

 signals:
  void ValueChanged(qint64 value);
  void ScrubStarted();
  void ScrubFinished(qint64 value);
  // The index refers to the position-sorted markers() collection.
  void MarkerActivated(int index);

 protected:
  void keyPressEvent(QKeyEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void paintEvent(QPaintEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

 private:
  QRectF TrackRect() const;
  qint64 PositionFromX(qreal x) const;
  qreal XFromPosition(qint64 position) const;
  void RebuildMarkerPositions();
  int HitMarker(qreal x) const;
  void UpdateAccessibleValue();

  QVector<TimelineMarker> markers_;
  QVector<qreal> marker_positions_;
  style::Tone tone_ = style::Tone::Shell;
  qint64 minimum_ = 0;
  qint64 maximum_ = 100;
  qint64 value_ = 0;
  bool dragging_ = false;
};

}  // namespace autoviz::widget
