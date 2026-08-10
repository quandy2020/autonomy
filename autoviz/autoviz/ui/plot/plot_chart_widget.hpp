/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QEnterEvent>
#include <QPoint>
#include <QRect>
#include <QVector>
#include <QWidget>

#include <optional>
#include <vector>

#include "autoviz/ui/plot/plot_types.hpp"

namespace autoviz {
namespace plot {

struct PlotInspectPoint {
  bool valid = false;
  double x = 0.0;
  double y = 0.0;
  QString series_label;
  QColor color;
};

class PlotChartWidget : public QWidget {
  Q_OBJECT

 public:
  explicit PlotChartWidget(QWidget* parent = nullptr);

  void setSeries(const std::vector<const PlotSeriesRuntime*>& series);
  void setXWindowSec(double seconds);
  void setXAxisMode(PlotXAxisMode mode);
  void setLockAxisScales(bool locked);
  void setReferenceTimeSec(double time_sec);
  void setInteractionMode(PlotInteractionMode mode);
  PlotInteractionMode interactionMode() const { return interaction_mode_; }
  void resetView();
  void applySyncedXRange(double min_x, double max_x);
  void clearInspection();
  /** Handles wheel zoom even when forwarded from a parent widget. */
  void handleWheelZoomAt(const QPointF& local_position, int angle_delta,
                         Qt::KeyboardModifiers modifiers);
  QRect legendAnchorRect() const;
  QVector<PlotValueRow> valueRowsAtX(double x) const;
  bool hoverActive() const { return hover_active_; }
  double hoverX() const { return hover_x_; }

 signals:
  void addSeriesRequested();
  void geometryChanged();
  void seekRequested(double timestamp_sec);
  void zoomToolToggleRequested();
  void viewRangeChanged(double min_x, double max_x, double min_y, double max_y);
  void hoverStateChanged();
  void inspectEscapeRequested();

 protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void enterEvent(QEnterEvent* event) override;
  void leaveEvent(QEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void mouseDoubleClickEvent(QMouseEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;

 private:
  struct AxisRange {
    double min_x = 0.0;
    double max_x = 30.0;
    double min_y = 0.0;
    double max_y = 1.0;
  };

  struct HoverSample {
    bool exact = false;
    double y = 0.0;
    double sample_x = 0.0;
  };

  struct TooltipRow {
    QString label;
    QColor color;
    QString value_text;
    bool latched = false;
  };

  AxisRange baseAxisRange() const;
  AxisRange applyLockedAxisScales(const AxisRange& range) const;
  AxisRange effectiveAxisRange() const;
  QRect chartRect() const;
  QRect addSeriesButtonRect() const;
  QPointF mapToChart(double x, double y, const AxisRange& range) const;
  std::optional<double> mapFromChartX(int pixel_x, const AxisRange& range) const;
  std::optional<double> mapFromChartY(int pixel_y, const AxisRange& range) const;
  std::optional<HoverSample> sampleAtX(const PlotSeriesRuntime& runtime,
                                       double x) const;
  std::optional<PlotInspectPoint> pickInspectPoint(const QPoint& pos,
                                                   const AxisRange& range) const;
  QString formatXAxisValue(double x) const;
  QString formatTooltipXValue(double x) const;
  QString formatDeltaXValue(double delta) const;
  QVector<TooltipRow> buildTooltipRows(double x) const;
  void updateHoverFromMouse(const QPoint& pos);
  void clearHover();
  void emitViewRangeIfNeeded();
  void drawGrid(QPainter& painter, const AxisRange& range,
                const QRect& chart) const;
  void drawSeries(QPainter& painter, const AxisRange& range,
                  const QRect& chart) const;
  void drawPlaybackLine(QPainter& painter, const AxisRange& range,
                        const QRect& chart) const;
  void drawHoverCrosshair(QPainter& painter, const AxisRange& range,
                          const QRect& chart) const;
  void drawInspection(QPainter& painter, const AxisRange& range,
                      const QRect& chart) const;
  void drawTooltip(QPainter& painter) const;
  void applyWheelZoom(const QPointF& position, int angle_delta,
                      Qt::KeyboardModifiers modifiers);
  void applyZoomRect(const QRect& selection, const AxisRange& range,
                     const QRect& chart);
  void beginPanGesture(const QPoint& pos, bool active_immediately);
  void updatePanGesture(const QPoint& pos);
  void finishPanGesture(bool emit_sync);
  bool exceedsDragThreshold(const QPoint& pos) const;

  std::vector<const PlotSeriesRuntime*> series_;
  double x_window_sec_ = 30.0;
  PlotXAxisMode x_axis_mode_ = PlotXAxisMode::kTimestamp;
  bool lock_axis_scales_ = false;
  double reference_time_sec_ = 0.0;
  PlotInteractionMode interaction_mode_ = PlotInteractionMode::kSelect;
  bool view_override_active_ = false;
  bool suppress_view_sync_ = false;
  double view_min_x_ = 0.0;
  double view_max_x_ = 30.0;
  double view_min_y_ = 0.0;
  double view_max_y_ = 1.0;
  bool dragging_ = false;
  bool pan_pending_ = false;
  QPoint drag_start_;
  AxisRange drag_start_range_;
  QRect zoom_rect_;
  bool hover_active_ = false;
  QPoint hover_pos_;
  double hover_x_ = 0.0;
  QVector<TooltipRow> tooltip_rows_;
  PlotInspectPoint inspect_a_;
  PlotInspectPoint inspect_b_;
};

}  // namespace plot
}  // namespace autoviz
