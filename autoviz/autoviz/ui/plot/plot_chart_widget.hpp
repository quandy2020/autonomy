/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plot_chart_widget.hpp
 * @brief Interactive 2D plot canvas: series, axes, pan/zoom/inspect, hover.
 *
 * Core chart used by @ref PlotPanel. Draws time/index/message-path series,
 * playback cursor, tooltips, and dual inspect points. Supports synced X-range
 * via @ref applySyncedXRange().
 *
 * @see PlotPanel
 * @see PlotSeriesRuntime
 * @see PlotViewSync
 */

#pragma once

#include <QColor>
#include <QContextMenuEvent>
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

/**
 * @struct PlotInspectPoint
 * @brief One latched inspect sample (A or B) on a series.
 */
struct PlotInspectPoint {
  bool valid = false;     /**< Whether this inspect slot is set. */
  double x = 0.0;         /**< Sample X (timestamp / index / path). */
  double y = 0.0;         /**< Sample Y value. */
  QString series_label;   /**< Series display label. */
  QColor color;           /**< Series color for markers / delta text. */
};

/**
 * @struct BrushSeriesStats
 * @brief Interval statistics for one series inside a brush X-range.
 */
struct BrushSeriesStats {
  QString label;   /**< Series label. */
  QColor color;    /**< Series color. */
  int count = 0;   /**< Number of samples in range. */
  double min_y = 0.0;
  double max_y = 0.0;
  double mean = 0.0;
  double stddev = 0.0;
  double rms = 0.0;
  double min_x = 0.0;
  double max_x = 0.0;
};

/**
 * @class PlotChartWidget
 * @brief Custom-painted chart with pan, box-zoom, select, inspect, and brush.
 *
 * ## Interaction modes (@ref PlotInteractionMode)
 *
 * - **Pan:** drag to translate the view
 * - **Zoom:** drag a rectangle to zoom; wheel zooms at cursor
 * - **Select:** click “+” to add series; seek on click when playback-linked
 * - **Inspect:** place A/B points and show Δx / Δy
 * - **Brush:** drag an X-range band; show min/max/mean/std/RMS/n/Δt
 *
 * @note Series pointers are non-owning; @ref PlotPanel owns the runtimes.
 */
class PlotChartWidget : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs an empty chart with default axis window.
   *
   * @param parent Qt parent widget.
   */
  explicit PlotChartWidget(QWidget* parent = nullptr);

  /**
   * @brief Replaces the series list drawn by the chart.
   *
   * @param series Non-owning pointers to enabled runtimes.
   */
  void setSeries(const std::vector<const PlotSeriesRuntime*>& series);

  /**
   * @brief Sets the rolling X window length in seconds (timestamp mode).
   *
   * @param seconds Window width.
   */
  void setXWindowSec(double seconds);

  /**
   * @brief Sets how the X axis is interpreted.
   *
   * @param mode Timestamp, index, or message-path mode.
   */
  void setXAxisMode(PlotXAxisMode mode);

  /**
   * @brief When locked, keeps axis scales from auto-fitting new data.
   *
   * @param locked @c true to freeze scales after user zoom/pan.
   */
  void setLockAxisScales(bool locked);

  /**
   * @brief Enables or disables automatic Y-axis fitting.
   *
   * @param enabled @c true to auto-fit Y from data; @c false to use fixed bounds.
   */
  void setYAutoScale(bool enabled);

  /**
   * @brief Sets the fixed Y-axis range used when auto-scale is off.
   *
   * @param min_y Minimum Y.
   * @param max_y Maximum Y.
   */
  void setYRange(double min_y, double max_y);

  /**
   * @brief Enables or disables automatic right Y-axis fitting.
   *
   * @param enabled @c true to auto-fit right Y from right-axis series.
   */
  void setYAutoScaleRight(bool enabled);

  /**
   * @brief Sets the fixed right Y-axis range used when auto-scale is off.
   *
   * @param min_y Minimum right Y.
   * @param max_y Maximum right Y.
   */
  void setYRangeRight(double min_y, double max_y);

  /**
   * @brief Shows or hides chart grid lines (axis ticks remain).
   *
   * @param visible @c true to draw grid.
   */
  void setShowGrid(bool visible);

  /**
   * @brief Shows or hides a horizontal reference line at @ref setReferenceY.
   *
   * @param visible @c true to draw the reference line.
   */
  void setShowReferenceY(bool visible);

  /**
   * @brief Sets the Y value for the horizontal reference line.
   *
   * @param y Reference Y in data units.
   */
  void setReferenceY(double y);

  /**
   * @brief Sets the playback / reference time drawn as a vertical line.
   *
   * @param time_sec Reference time in seconds.
   */
  void setReferenceTimeSec(double time_sec);

  /**
   * @brief Selects the mouse interaction tool.
   *
   * @param mode Pan, zoom, select, or inspect.
   */
  void setInteractionMode(PlotInteractionMode mode);

  /**
   * @brief Returns the current interaction mode.
   *
   * @return Active @ref PlotInteractionMode.
   */
  PlotInteractionMode interactionMode() const { return interaction_mode_; }

  /**
   * @brief Clears view overrides and returns to auto-fitting axes.
   */
  void resetView();

  /**
   * @brief Applies an X range from @ref PlotViewSync without re-broadcasting.
   *
   * @param min_x Synced minimum X.
   * @param max_x Synced maximum X.
   */
  void applySyncedXRange(double min_x, double max_x);

  /**
   * @brief Clears latched inspect points A/B.
   */
  void clearInspection();

  /**
   * @brief Clears the latched brush selection and stats overlay.
   */
  void clearBrush();

  /**
   * @brief Handles wheel zoom even when forwarded from a parent widget.
   *
   * @param local_position Cursor position in chart coordinates.
   * @param angle_delta Qt wheel angle delta.
   * @param modifiers Keyboard modifiers (e.g. axis-constrained zoom).
   */
  void handleWheelZoomAt(const QPointF& local_position, int angle_delta,
                         Qt::KeyboardModifiers modifiers);

  /**
   * @brief Screen rect used to anchor the floating legend.
   *
   * @return Legend anchor rectangle in widget coordinates.
   */
  QRect legendAnchorRect() const;

  /**
   * @brief Samples all series at X for legend value rows.
   *
   * @param x Query X coordinate.
   * @return One row per series (latched when no sample at @p x).
   */
  QVector<PlotValueRow> valueRowsAtX(double x) const;

  /**
   * @brief Whether the hover crosshair is active.
   *
   * @return @c true while the cursor is over the chart with a valid X.
   */
  bool hoverActive() const { return hover_active_; }

  /**
   * @brief Current hover X when @ref hoverActive() is true.
   *
   * @return Hover X coordinate.
   */
  double hoverX() const { return hover_x_; }

 signals:
  /** User clicked the in-chart “add series” affordance. */
  void addSeriesRequested();

  /** Chart geometry / legend anchor changed (e.g. resize). */
  void geometryChanged();

  /**
   * @brief Request seeking playback to a timestamp.
   *
   * @param timestamp_sec Seek target in seconds.
   */
  void seekRequested(double timestamp_sec);

  /** Escape / gesture requesting the zoom tool be toggled off. */
  void zoomToolToggleRequested();

  /**
   * @brief Emitted when the visible axis range changes (for plot sync).
   *
   * @param min_x Visible minimum X.
   * @param max_x Visible maximum X.
   * @param min_y Visible minimum Y.
   * @param max_y Visible maximum Y.
   */
  void viewRangeChanged(double min_x, double max_x, double min_y, double max_y);

  /** Hover crosshair appeared, moved, or cleared. */
  void hoverStateChanged();

  /** Escape pressed while inspecting — clear / exit inspect. */
  void inspectEscapeRequested();

 protected:
  /** @brief Paints grid, series, playback line, hover, inspect, tooltip. */
  void paintEvent(QPaintEvent* event) override;

  /** @brief Starts pan/zoom/select/inspect gestures. */
  void mousePressEvent(QMouseEvent* event) override;

  /** @brief Copy hover values or the nearest series path. */
  void contextMenuEvent(QContextMenuEvent* event) override;

  /** @brief Updates pan, zoom rect, hover, or inspect preview. */
  void mouseMoveEvent(QMouseEvent* event) override;

  /** @brief Finishes pan/zoom gestures. */
  void mouseReleaseEvent(QMouseEvent* event) override;

  /** @brief Enables hover tracking on enter. */
  void enterEvent(QEnterEvent* event) override;

  /** @brief Clears hover on leave. */
  void leaveEvent(QEvent* event) override;

  /** @brief Wheel zoom at cursor. */
  void wheelEvent(QWheelEvent* event) override;

  /** @brief Double-click resets the view. */
  void mouseDoubleClickEvent(QMouseEvent* event) override;

  /** @brief Emits @ref geometryChanged() on resize. */
  void resizeEvent(QResizeEvent* event) override;

  /** @brief Escape clears inspect / exits zoom as appropriate. */
  void keyPressEvent(QKeyEvent* event) override;

 private:
  /**
   * @struct AxisRange
   * @brief Axis-aligned data bounds for mapping to pixels.
   */
  struct AxisRange {
    double min_x = 0.0;
    double max_x = 30.0;
    double min_y = 0.0;
    double max_y = 1.0;
    double min_y_right = 0.0;
    double max_y_right = 1.0;
    bool has_right_y = false;
  };

  /**
   * @struct HoverSample
   * @brief Y sample under the hover X for one series.
   */
  struct HoverSample {
    bool exact = false;     /**< True when an exact X match exists. */
    double y = 0.0;         /**< Sampled / interpolated Y. */
    double sample_x = 0.0;  /**< Actual sample X used. */
  };

  /**
   * @struct TooltipRow
   * @brief One line in the hover tooltip.
   */
  struct TooltipRow {
    QString label;       /**< Series label. */
    QColor color;        /**< Series color swatch. */
    QString value_text;  /**< Formatted Y (and latch marker). */
    bool latched = false; /**< True when showing last-known value. */
  };

  /** Auto-fit range from series data and X window. */
  AxisRange baseAxisRange() const;

  /** Apply lock / view-override on top of @p range. */
  AxisRange applyLockedAxisScales(const AxisRange& range) const;

  /** Range actually used for the current paint / hit-test. */
  AxisRange effectiveAxisRange() const;

  /** Inner plot rectangle excluding margins/axes. */
  QRect chartRect() const;

  /** Hit target for the “+ add series” button. */
  QRect addSeriesButtonRect() const;

  /** Maps data (x,y) to widget pixels on the left or right Y-axis. */
  QPointF mapToChart(double x, double y, const AxisRange& range,
                     bool use_right_y = false) const;

  /** Inverse map for pixel X → data X. */
  std::optional<double> mapFromChartX(int pixel_x, const AxisRange& range) const;

  /** Inverse map for pixel Y → data Y. */
  std::optional<double> mapFromChartY(int pixel_y, const AxisRange& range) const;

  /** Sample one series at X (nearest / interpolate). */
  std::optional<HoverSample> sampleAtX(const PlotSeriesRuntime& runtime,
                                       double x) const;

  /** Pick the nearest series point under @p pos for inspect. */
  std::optional<PlotInspectPoint> pickInspectPoint(const QPoint& pos,
                                                   const AxisRange& range) const;

  /** Format an axis tick label for X. */
  QString formatXAxisValue(double x) const;

  /** Format X for the hover tooltip header. */
  QString formatTooltipXValue(double x) const;

  /** Format Δx between inspect points. */
  QString formatDeltaXValue(double delta) const;

  /** Build tooltip rows at hover X. */
  QVector<TooltipRow> buildTooltipRows(double x) const;

  /** Update hover state from a mouse position. */
  void updateHoverFromMouse(const QPoint& pos);

  /** Clear hover crosshair and tooltip. */
  void clearHover();

  /** Emit @ref viewRangeChanged() when the visible range actually changed. */
  void emitViewRangeIfNeeded();

  /** Draw background grid and axis ticks. */
  void drawGrid(QPainter& painter, const AxisRange& range,
                const QRect& chart) const;

  /** Stroke all series polylines. */
  void drawSeries(QPainter& painter, const AxisRange& range,
                  const QRect& chart) const;

  /** Draw the playback / reference vertical line. */
  void drawPlaybackLine(QPainter& painter, const AxisRange& range,
                        const QRect& chart) const;

  /** Draw hover crosshair. */
  void drawHoverCrosshair(QPainter& painter, const AxisRange& range,
                          const QRect& chart) const;

  /** Draw inspect A/B markers and delta readouts. */
  void drawInspection(QPainter& painter, const AxisRange& range,
                      const QRect& chart) const;

  /** Draw brush X-range band and statistics overlay. */
  void drawBrush(QPainter& painter, const AxisRange& range,
                 const QRect& chart) const;

  /** Draw horizontal reference line when enabled. */
  void drawReferenceY(QPainter& painter, const AxisRange& range,
                      const QRect& chart) const;

  /** Draw the hover tooltip box. */
  void drawTooltip(QPainter& painter) const;

  /** Apply wheel zoom centered at @p position. */
  void applyWheelZoom(const QPointF& position, int angle_delta,
                      Qt::KeyboardModifiers modifiers);

  /** Commit a box-zoom from @p selection. */
  void applyZoomRect(const QRect& selection, const AxisRange& range,
                     const QRect& chart);

  /** Latch brush X-range from a pixel span and recompute stats. */
  void applyBrushSpan(int pixel_a, int pixel_b, const AxisRange& range,
                      const QRect& chart);

  /** Recompute @ref brush_stats_ for the current brush X-range. */
  void recomputeBrushStats();

  /** Start a pan gesture (optionally pending until drag threshold). */
  void beginPanGesture(const QPoint& pos, bool active_immediately);

  /** Update pan translation from cursor motion. */
  void updatePanGesture(const QPoint& pos);

  /** Finish pan and optionally emit view sync. */
  void finishPanGesture(bool emit_sync);

  /** Whether @p pos has moved beyond the click/drag threshold. */
  bool exceedsDragThreshold(const QPoint& pos) const;

  std::vector<const PlotSeriesRuntime*> series_;  /**< Non-owning series list. */
  double x_window_sec_ = 30.0;  /**< Rolling X window (timestamp mode). */
  PlotXAxisMode x_axis_mode_ = PlotXAxisMode::kTimestamp;  /**< X-axis mode. */
  bool lock_axis_scales_ = false;  /**< Freeze auto-fit when set. */
  bool y_auto_scale_ = true;  /**< Auto-fit Y from data when unlocked. */
  double y_min_ = 0.0;  /**< Fixed Y min when auto-scale is off. */
  double y_max_ = 1.0;  /**< Fixed Y max when auto-scale is off. */
  bool y_auto_scale_right_ = true;  /**< Auto-fit right Y. */
  double y_min_right_ = 0.0;  /**< Fixed right Y min. */
  double y_max_right_ = 1.0;  /**< Fixed right Y max. */
  bool show_grid_ = true;  /**< Draw grid lines. */
  bool show_reference_y_ = false;  /**< Draw horizontal reference line. */
  double reference_y_ = 0.0;  /**< Reference line Y value. */
  double reference_time_sec_ = 0.0;  /**< Playback cursor time. */
  PlotInteractionMode interaction_mode_ = PlotInteractionMode::kSelect;  /**< Tool. */
  bool view_override_active_ = false;  /**< User zoom/pan overrides auto range. */
  bool suppress_view_sync_ = false;  /**< Block re-entrant sync publishes. */
  double view_min_x_ = 0.0;   /**< Override min X. */
  double view_max_x_ = 30.0;  /**< Override max X. */
  double view_min_y_ = 0.0;   /**< Override min Y. */
  double view_max_y_ = 1.0;   /**< Override max Y. */
  bool dragging_ = false;     /**< Active drag gesture. */
  bool pan_pending_ = false;  /**< Pressed but not yet past drag threshold. */
  QPoint drag_start_;         /**< Gesture start pixel. */
  AxisRange drag_start_range_; /**< Axis range at gesture start. */
  QRect zoom_rect_;           /**< Current box-zoom rectangle. */
  bool hover_active_ = false; /**< Hover crosshair visible. */
  QPoint hover_pos_;          /**< Hover pixel position. */
  double hover_x_ = 0.0;      /**< Hover data X. */
  QVector<TooltipRow> tooltip_rows_;  /**< Cached tooltip contents. */
  PlotInspectPoint inspect_a_;  /**< Inspect point A. */
  PlotInspectPoint inspect_b_;  /**< Inspect point B. */
  bool brush_active_ = false;   /**< Latched brush selection present. */
  bool brush_dragging_ = false; /**< Brush drag in progress. */
  double brush_min_x_ = 0.0;    /**< Brush range min X. */
  double brush_max_x_ = 0.0;    /**< Brush range max X. */
  int brush_drag_x_ = 0;        /**< Live brush drag end pixel X. */
  QVector<BrushSeriesStats> brush_stats_;  /**< Stats for latched brush. */
};

}  // namespace plot
}  // namespace autoviz
