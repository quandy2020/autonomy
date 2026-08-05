/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <optional>

#include <QColor>
#include <QPoint>
#include <QRect>
#include <QWidget>

#include <vector>

#include "autoviz/ui/state_transitions/state_transition_types.hpp"

namespace autoviz {
namespace state_transitions {

class StateTransitionViewWidget : public QWidget {
  Q_OBJECT

 public:
  explicit StateTransitionViewWidget(QWidget* parent = nullptr);

  void setSeries(const std::vector<const StateTransitionSeriesRuntime*>& series);
  void setPanelConfig(const StateTransitionPanelConfig& config);
  void setReferenceTimeSec(double time_sec);
  void resetView();
  void setBoxZoomMode(bool enabled);
  bool boxZoomMode() const { return box_zoom_mode_; }

 signals:
  void addSeriesRequested();
  void seekRequested(double timestamp_sec);
  void boxZoomModeChanged(bool enabled);

 protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

 private:
  struct TimeRange {
    double min_time = 0.0;
    double max_time = 30.0;
  };

  TimeRange defaultTimeRange() const;
  TimeRange effectiveTimeRange() const;
  QRect contentRect() const;
  QRect labelColumnRect() const;
  QRect chartRect() const;
  QRect addSeriesButtonRect() const;
  int visibleRowCount() const;
  int headerHeight() const { return 18; }
  int rowHeight() const { return 26; }
  int labelColumnWidth() const { return 108; }
  int timeAxisHeight() const { return 20; }

  QRect fullRowRect(int visible_row_index) const;
  QRect chartRowRect(int visible_row_index) const;
  double mapTimeToX(double time_sec, const TimeRange& range,
                    const QRect& chart) const;
  std::optional<double> mapXToTime(int x, const TimeRange& range,
                                   const QRect& chart) const;
  void drawHeader(QPainter& painter) const;
  void drawLabelColumnBackground(QPainter& painter) const;
  void drawEmptyState(QPainter& painter, const QRect& chart) const;
  void drawTimeAxis(QPainter& painter, const TimeRange& range,
                    const QRect& chart) const;
  void drawSeriesRows(QPainter& painter, const TimeRange& range,
                      const QRect& chart) const;
  void drawAddSeriesButton(QPainter& painter) const;
  void drawBoxZoomOverlay(QPainter& painter) const;
  std::size_t lowerSegmentBound(const std::deque<StateSegment>& segments,
                                double min_time) const;
  void clampRowScroll();

  std::vector<const StateTransitionSeriesRuntime*> series_;
  StateTransitionPanelConfig config_;
  double reference_time_sec_ = 0.0;
  bool has_custom_view_ = false;
  double view_min_time_ = 0.0;
  double view_max_time_ = 30.0;
  int row_scroll_offset_ = 0;
  bool panning_ = false;
  QPoint pan_anchor_;
  double pan_anchor_min_time_ = 0.0;
  int pan_anchor_row_scroll_ = 0;
  bool box_zoom_mode_ = false;
  bool box_zoom_dragging_ = false;
  QPoint box_zoom_start_;
  QPoint box_zoom_end_;
};

}  // namespace state_transitions
}  // namespace autoviz
