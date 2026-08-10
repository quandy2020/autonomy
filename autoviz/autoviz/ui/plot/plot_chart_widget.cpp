/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/plot_chart_widget.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QKeyEvent>
#include <QLineF>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QEnterEvent>
#include <QtMath>

namespace autoviz {
namespace plot {
namespace {

constexpr int kAxisLeft = 56;
constexpr int kAxisTop = 16;
constexpr int kAxisRight = 16;
constexpr int kAxisBottom = 48;
constexpr double kExactSampleEpsilon = 1e-6;
constexpr double kInspectPickRadiusPx = 10.0;
constexpr int kPanDragThresholdPx = 4;

}  // namespace

PlotChartWidget::PlotChartWidget(QWidget* parent) : QWidget(parent) {
  setMinimumSize(320, 220);
  setMouseTracking(true);
  setFocusPolicy(Qt::StrongFocus);
  setAutoFillBackground(true);
  QPalette palette = this->palette();
  palette.setColor(QPalette::Window, QColor(250, 250, 250));
  setPalette(palette);
}

void PlotChartWidget::setSeries(const std::vector<const PlotSeriesRuntime*>& series) {
  series_ = series;
  if (hover_active_) {
    tooltip_rows_ = buildTooltipRows(hover_x_);
  }
  update();
}

void PlotChartWidget::setXWindowSec(double seconds) {
  x_window_sec_ = std::max(1.0, seconds);
  update();
}

void PlotChartWidget::setXAxisMode(PlotXAxisMode mode) {
  x_axis_mode_ = mode;
  update();
}

void PlotChartWidget::setLockAxisScales(bool locked) {
  lock_axis_scales_ = locked;
  update();
}

void PlotChartWidget::setReferenceTimeSec(double time_sec) {
  reference_time_sec_ = time_sec;
  update();
}

void PlotChartWidget::setInteractionMode(PlotInteractionMode mode) {
  interaction_mode_ = mode;
  dragging_ = false;
  pan_pending_ = false;
  zoom_rect_ = QRect();
  switch (mode) {
    case PlotInteractionMode::kPan:
      setCursor(Qt::OpenHandCursor);
      break;
    case PlotInteractionMode::kZoom:
      setCursor(Qt::CrossCursor);
      break;
    case PlotInteractionMode::kInspect:
      setCursor(Qt::PointingHandCursor);
      break;
    case PlotInteractionMode::kSelect:
    default:
      setCursor(hover_active_ ? Qt::CrossCursor : Qt::ArrowCursor);
      break;
  }
  if (mode != PlotInteractionMode::kInspect) {
    clearInspection();
  }
  update();
}

void PlotChartWidget::applySyncedXRange(double min_x, double max_x) {
  suppress_view_sync_ = true;
  view_min_x_ = min_x;
  view_max_x_ = max_x;
  view_override_active_ = true;
  suppress_view_sync_ = false;
  update();
}

void PlotChartWidget::clearInspection() {
  inspect_a_ = PlotInspectPoint{};
  inspect_b_ = PlotInspectPoint{};
  update();
}

void PlotChartWidget::emitViewRangeIfNeeded() {
  if (suppress_view_sync_ || !view_override_active_) {
    return;
  }
  emit viewRangeChanged(view_min_x_, view_max_x_, view_min_y_, view_max_y_);
}

void PlotChartWidget::resetView() {
  view_override_active_ = false;
  dragging_ = false;
  pan_pending_ = false;
  zoom_rect_ = QRect();
  update();
}

bool PlotChartWidget::exceedsDragThreshold(const QPoint& pos) const {
  return (pos - drag_start_).manhattanLength() >= kPanDragThresholdPx;
}

void PlotChartWidget::beginPanGesture(const QPoint& pos, bool active_immediately) {
  drag_start_ = pos;
  drag_start_range_ = effectiveAxisRange();
  pan_pending_ = !active_immediately;
  dragging_ = active_immediately;
  if (dragging_) {
    setCursor(Qt::ClosedHandCursor);
  }
}

void PlotChartWidget::updatePanGesture(const QPoint& pos) {
  if (pan_pending_ && exceedsDragThreshold(pos)) {
    pan_pending_ = false;
    dragging_ = true;
    setCursor(Qt::ClosedHandCursor);
  }
  if (!dragging_) {
    return;
  }

  const QRect chart = chartRect();
  const double dx = drag_start_range_.max_x - drag_start_range_.min_x;
  const double dy = drag_start_range_.max_y - drag_start_range_.min_y;
  const double px = chart.width() <= 0 ? 0.0 : dx / chart.width();
  const double py = chart.height() <= 0 ? 0.0 : dy / chart.height();
  const int delta_x = pos.x() - drag_start_.x();
  const int delta_y = pos.y() - drag_start_.y();
  view_min_x_ = drag_start_range_.min_x - delta_x * px;
  view_max_x_ = drag_start_range_.max_x - delta_x * px;
  view_min_y_ = drag_start_range_.min_y + delta_y * py;
  view_max_y_ = drag_start_range_.max_y + delta_y * py;
  view_override_active_ = true;
  update();
}

void PlotChartWidget::finishPanGesture(bool emit_sync) {
  const bool was_dragging = dragging_;
  dragging_ = false;
  pan_pending_ = false;
  if (interaction_mode_ == PlotInteractionMode::kPan) {
    setCursor(Qt::OpenHandCursor);
  } else if (interaction_mode_ == PlotInteractionMode::kSelect && hover_active_) {
    setCursor(Qt::CrossCursor);
  } else {
    unsetCursor();
  }
  if (was_dragging && emit_sync) {
    emitViewRangeIfNeeded();
  }
}

QRect PlotChartWidget::chartRect() const {
  return QRect(kAxisLeft, kAxisTop, width() - kAxisLeft - kAxisRight,
               height() - kAxisTop - kAxisBottom);
}

QRect PlotChartWidget::addSeriesButtonRect() const {
  const QRect chart = chartRect();
  return QRect(chart.left() + 8, chart.top() + 8, 170, 28);
}

QRect PlotChartWidget::legendAnchorRect() const {
  const QRect button = addSeriesButtonRect();
  return QRect(button.left(), button.bottom() + 8, 180, 120);
}

PlotChartWidget::AxisRange PlotChartWidget::baseAxisRange() const {
  AxisRange range;
  range.max_x = x_axis_mode_ == PlotXAxisMode::kTimestamp ? reference_time_sec_
                                                        : 30.0;
  range.min_x = x_axis_mode_ == PlotXAxisMode::kTimestamp
                    ? range.max_x - x_window_sec_
                    : 0.0;
  range.min_y = 0.0;
  range.max_y = 1.0;

  bool has_points = false;
  for (const PlotSeriesRuntime* runtime_ptr : series_) {
    if (runtime_ptr == nullptr) {
      continue;
    }
    const PlotSeriesRuntime& runtime = *runtime_ptr;
    if (!runtime.config.enabled || !runtime.config.show_line) {
      continue;
    }
    for (const PlotPoint& point : runtime.points) {
      if (!has_points) {
        range.min_x = point.x;
        range.max_x = point.x;
        range.min_y = point.y;
        range.max_y = point.y;
        has_points = true;
      } else {
        range.min_x = std::min(range.min_x, point.x);
        range.max_x = std::max(range.max_x, point.x);
        range.min_y = std::min(range.min_y, point.y);
        range.max_y = std::max(range.max_y, point.y);
      }
    }
  }

  if (!has_points) {
    return range;
  }

  if (x_axis_mode_ == PlotXAxisMode::kTimestamp && !view_override_active_) {
    range.max_x = reference_time_sec_;
    range.min_x = reference_time_sec_ - x_window_sec_;
  }

  if (qFuzzyCompare(range.min_y, range.max_y)) {
    range.min_y -= 0.5;
    range.max_y += 0.5;
  } else {
    const double padding = (range.max_y - range.min_y) * 0.1;
    range.min_y -= padding;
    range.max_y += padding;
  }
  return applyLockedAxisScales(range);
}

PlotChartWidget::AxisRange PlotChartWidget::applyLockedAxisScales(
    const AxisRange& range) const {
  if (!lock_axis_scales_) {
    return range;
  }
  const QRect chart = chartRect();
  if (chart.width() <= 0 || chart.height() <= 0) {
    return range;
  }
  AxisRange locked = range;
  const double x_span = std::max(locked.max_x - locked.min_x, 1e-9);
  const double y_span = std::max(locked.max_y - locked.min_y, 1e-9);
  const double cx = (locked.min_x + locked.max_x) * 0.5;
  const double cy = (locked.min_y + locked.max_y) * 0.5;
  const double y_span_for_lock = x_span * chart.height() / chart.width();
  const double x_span_for_lock = y_span * chart.width() / chart.height();
  if (y_span_for_lock > y_span) {
    locked.min_y = cy - y_span_for_lock * 0.5;
    locked.max_y = cy + y_span_for_lock * 0.5;
  }
  if (x_span_for_lock > x_span) {
    locked.min_x = cx - x_span_for_lock * 0.5;
    locked.max_x = cx + x_span_for_lock * 0.5;
  }
  return locked;
}

PlotChartWidget::AxisRange PlotChartWidget::effectiveAxisRange() const {
  if (view_override_active_) {
    AxisRange range;
    range.min_x = view_min_x_;
    range.max_x = view_max_x_;
    range.min_y = view_min_y_;
    range.max_y = view_max_y_;
    return applyLockedAxisScales(range);
  }
  return baseAxisRange();
}

QPointF PlotChartWidget::mapToChart(double x, double y,
                                    const AxisRange& range) const {
  const QRect chart = chartRect();
  const double dx = range.max_x - range.min_x;
  const double dy = range.max_y - range.min_y;
  const double nx = dx <= 0.0 ? 0.0 : (x - range.min_x) / dx;
  const double ny = dy <= 0.0 ? 0.0 : (y - range.min_y) / dy;
  return QPointF(chart.left() + nx * chart.width(),
                 chart.bottom() - ny * chart.height());
}

std::optional<double> PlotChartWidget::mapFromChartX(
    int pixel_x, const AxisRange& range) const {
  const QRect chart = chartRect();
  if (chart.width() <= 0) {
    return std::nullopt;
  }
  const double ratio =
      static_cast<double>(pixel_x - chart.left()) / chart.width();
  const double dx = range.max_x - range.min_x;
  return range.min_x + dx * ratio;
}

std::optional<double> PlotChartWidget::mapFromChartY(
    int pixel_y, const AxisRange& range) const {
  const QRect chart = chartRect();
  if (chart.height() <= 0) {
    return std::nullopt;
  }
  const double ratio =
      static_cast<double>(chart.bottom() - pixel_y) / chart.height();
  const double dy = range.max_y - range.min_y;
  return range.min_y + dy * ratio;
}

std::optional<PlotChartWidget::HoverSample> PlotChartWidget::sampleAtX(
    const PlotSeriesRuntime& runtime, double x) const {
  if (runtime.points.empty()) {
    return std::nullopt;
  }

  HoverSample best;
  bool found = false;
  for (const PlotPoint& point : runtime.points) {
    if (point.x > x) {
      break;
    }
    best.y = point.y;
    best.sample_x = point.x;
    best.exact = std::abs(point.x - x) <= kExactSampleEpsilon;
    found = true;
  }

  if (!found) {
    return std::nullopt;
  }
  return best;
}

QString PlotChartWidget::formatXAxisValue(double x) const {
  if (x_axis_mode_ == PlotXAxisMode::kTimestamp) {
    const double rel = x - reference_time_sec_;
    return QString::number(rel, 'f', 1);
  }
  if (x_axis_mode_ == PlotXAxisMode::kIndex) {
    return QString::number(qRound64(x));
  }
  return QString::number(x, 'f', 1);
}

QString PlotChartWidget::formatTooltipXValue(double x) const {
  if (x_axis_mode_ == PlotXAxisMode::kTimestamp) {
    const double rel = x - reference_time_sec_;
    return tr("Time: %1 s").arg(QString::number(rel, 'f', 3));
  }
  if (x_axis_mode_ == PlotXAxisMode::kIndex) {
    return tr("Index: %1").arg(qRound64(x));
  }
  return tr("X: %1").arg(QString::number(x, 'f', 3));
}

QString PlotChartWidget::formatDeltaXValue(double delta) const {
  if (x_axis_mode_ == PlotXAxisMode::kTimestamp) {
    return tr("Δt: %1 s").arg(QString::number(delta, 'f', 3));
  }
  if (x_axis_mode_ == PlotXAxisMode::kIndex) {
    return tr("Δ index: %1").arg(qRound64(delta));
  }
  return tr("Δx: %1").arg(QString::number(delta, 'f', 3));
}

QVector<PlotValueRow> PlotChartWidget::valueRowsAtX(double x) const {
  QVector<PlotValueRow> rows;
  for (const PlotSeriesRuntime* runtime_ptr : series_) {
    if (runtime_ptr == nullptr || !runtime_ptr->config.enabled) {
      continue;
    }
    const PlotSeriesRuntime& runtime = *runtime_ptr;
    const std::optional<HoverSample> sample = sampleAtX(runtime, x);
    if (!sample.has_value()) {
      continue;
    }
    PlotValueRow row;
    row.color = runtime.config.color;
    row.label = runtime.config.label.isEmpty() ? runtime.config.field_path
                                                 : runtime.config.label;
    row.latched = !sample->exact;
    if (sample->exact) {
      row.value_text = QString::number(sample->y, 'g', 4);
    } else {
      const double delta = x - sample->sample_x;
      row.value_text = tr("%1 (%2s ago)")
                           .arg(QString::number(sample->y, 'g', 4))
                           .arg(QString::number(delta, 'f', 2));
    }
    rows.push_back(row);
  }
  return rows;
}

std::optional<PlotInspectPoint> PlotChartWidget::pickInspectPoint(
    const QPoint& pos, const AxisRange& range) const {
  PlotInspectPoint best;
  double best_dist = kInspectPickRadiusPx;
  for (const PlotSeriesRuntime* runtime_ptr : series_) {
    if (runtime_ptr == nullptr || !runtime_ptr->config.enabled) {
      continue;
    }
    const PlotSeriesRuntime& runtime = *runtime_ptr;
    for (const PlotPoint& point : runtime.points) {
      const QPointF mapped = mapToChart(point.x, point.y, range);
      const double dist = QLineF(mapped, pos).length();
      if (dist < best_dist) {
        best_dist = dist;
        best.valid = true;
        best.x = point.x;
        best.y = point.y;
        best.color = runtime.config.color;
        best.series_label = runtime.config.label.isEmpty() ? runtime.config.field_path
                                                           : runtime.config.label;
      }
    }
  }
  if (!best.valid) {
    return std::nullopt;
  }
  return best;
}

QVector<PlotChartWidget::TooltipRow> PlotChartWidget::buildTooltipRows(
    double x) const {
  QVector<TooltipRow> rows;
  rows.reserve(static_cast<int>(series_.size()) + 1);

  TooltipRow header;
  header.label = formatTooltipXValue(x);
  header.color = QColor(60, 60, 60);
  header.value_text = QString();
  rows.push_back(header);

  for (const PlotSeriesRuntime* runtime_ptr : series_) {
    if (runtime_ptr == nullptr || !runtime_ptr->config.enabled) {
      continue;
    }
    const PlotSeriesRuntime& runtime = *runtime_ptr;
    const std::optional<HoverSample> sample = sampleAtX(runtime, x);
    if (!sample.has_value()) {
      continue;
    }

    TooltipRow row;
    row.color = runtime.config.color;
    row.label = runtime.config.label.isEmpty() ? runtime.config.field_path
                                               : runtime.config.label;
    row.latched = !sample->exact;
    if (sample->exact) {
      row.value_text = QString::number(sample->y, 'g', 4);
    } else {
      const double delta = x - sample->sample_x;
      row.value_text =
          tr("%1 (%2s ago)")
              .arg(QString::number(sample->y, 'g', 4))
              .arg(QString::number(delta, 'f', 2));
    }
    rows.push_back(row);
  }
  return rows;
}

void PlotChartWidget::updateHoverFromMouse(const QPoint& pos) {
  const QRect chart = chartRect();
  if (!chart.contains(pos)) {
    clearHover();
    return;
  }

  const AxisRange range = effectiveAxisRange();
  const std::optional<double> data_x = mapFromChartX(pos.x(), range);
  if (!data_x.has_value()) {
    clearHover();
    return;
  }

  hover_active_ = true;
  hover_pos_ = pos;
  hover_x_ = *data_x;
  tooltip_rows_ = buildTooltipRows(hover_x_);
  if (interaction_mode_ == PlotInteractionMode::kSelect) {
    setCursor(Qt::CrossCursor);
  }
  emit hoverStateChanged();
  update();
}

void PlotChartWidget::clearHover() {
  if (!hover_active_) {
    return;
  }
  hover_active_ = false;
  tooltip_rows_.clear();
  if (interaction_mode_ == PlotInteractionMode::kSelect) {
    setCursor(Qt::ArrowCursor);
  }
  emit hoverStateChanged();
  update();
}

void PlotChartWidget::drawGrid(QPainter& painter, const AxisRange& range,
                               const QRect& chart) const {
  painter.setPen(QColor(210, 210, 210));
  painter.drawRect(chart);

  for (int i = 0; i <= 10; ++i) {
    const double y = range.min_y + (range.max_y - range.min_y) * i / 10.0;
    const QPointF p = mapToChart(range.min_x, y, range);
    painter.setPen(QColor(235, 235, 235));
    painter.drawLine(QPointF(chart.left(), p.y()), QPointF(chart.right(), p.y()));
    painter.setPen(QColor(120, 120, 120));
    painter.drawText(QRect(4, static_cast<int>(p.y()) - 8, 48, 16),
                     Qt::AlignRight | Qt::AlignVCenter,
                     QString::number(y, 'f', 2));
  }

  for (int i = 0; i <= 6; ++i) {
    const double x = range.min_x + (range.max_x - range.min_x) * i / 6.0;
    const QPointF p = mapToChart(x, range.min_y, range);
    painter.setPen(QColor(235, 235, 235));
    painter.drawLine(QPointF(p.x(), chart.top()), QPointF(p.x(), chart.bottom()));
    painter.setPen(QColor(120, 120, 120));
    painter.drawText(QRect(static_cast<int>(p.x()) - 24, chart.bottom() + 4, 48,
                           16),
                     Qt::AlignHCenter | Qt::AlignTop, formatXAxisValue(x));
  }
}

void PlotChartWidget::drawSeries(QPainter& painter, const AxisRange& range,
                                 const QRect& chart) const {
  if (series_.empty()) {
    return;
  }

  painter.save();
  painter.setClipRect(chart);

  struct HoverMarker {
    QPointF position;
    QColor color;
  };
  QVector<HoverMarker> hover_markers;

  const double x_margin = (range.max_x - range.min_x) * 0.01;

  for (const PlotSeriesRuntime* runtime_ptr : series_) {
    if (runtime_ptr == nullptr) {
      continue;
    }
    const PlotSeriesRuntime& runtime = *runtime_ptr;
    if (!runtime.config.enabled || !runtime.config.show_line ||
        runtime.points.size() < 2) {
      continue;
    }

    QPainterPath path;
    bool started = false;
    for (const PlotPoint& point : runtime.points) {
      if (point.x < range.min_x - x_margin || point.x > range.max_x + x_margin) {
        started = false;
        continue;
      }
      const QPointF mapped = mapToChart(point.x, point.y, range);
      if (!started) {
        path.moveTo(mapped);
        started = true;
      } else {
        path.lineTo(mapped);
      }
    }
    if (path.isEmpty()) {
      continue;
    }

    QPen pen(runtime.config.color, 2.0);
    pen.setCosmetic(true);
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);
    painter.drawPath(path);

    if (hover_active_ && interaction_mode_ == PlotInteractionMode::kSelect) {
      const std::optional<HoverSample> sample = sampleAtX(runtime, hover_x_);
      if (sample.has_value()) {
        hover_markers.push_back(
            HoverMarker{mapToChart(sample->sample_x, sample->y, range),
                        runtime.config.color});
      }
    }
  }

  for (const HoverMarker& marker : hover_markers) {
    painter.setPen(Qt::NoPen);
    painter.setBrush(marker.color);
    painter.drawEllipse(marker.position, 4.0, 4.0);
  }

  painter.restore();
}

void PlotChartWidget::drawPlaybackLine(QPainter& painter,
                                       const AxisRange& range,
                                       const QRect& chart) const {
  if (x_axis_mode_ != PlotXAxisMode::kTimestamp) {
    return;
  }
  if (reference_time_sec_ < range.min_x || reference_time_sec_ > range.max_x) {
    return;
  }

  const QPointF top = mapToChart(reference_time_sec_, range.max_y, range);
  const QPointF bottom = mapToChart(reference_time_sec_, range.min_y, range);
  QPen pen(QColor(130, 130, 130));
  pen.setWidth(2);
  pen.setCosmetic(true);
  painter.setPen(pen);
  painter.drawLine(QPointF(top.x(), chart.top()), QPointF(bottom.x(), chart.bottom()));
}

void PlotChartWidget::drawHoverCrosshair(QPainter& painter,
                                         const AxisRange& range,
                                         const QRect& chart) const {
  if (!hover_active_ || hover_x_ < range.min_x || hover_x_ > range.max_x) {
    return;
  }
  if (interaction_mode_ == PlotInteractionMode::kInspect) {
    return;
  }

  const QPointF top = mapToChart(hover_x_, range.max_y, range);
  QPen pen(QColor(244, 196, 48));
  pen.setWidth(2);
  pen.setCosmetic(true);
  pen.setStyle(interaction_mode_ == PlotInteractionMode::kSelect ? Qt::SolidLine
                                                                 : Qt::DashLine);
  painter.setPen(pen);
  painter.drawLine(QPointF(top.x(), chart.top()), QPointF(top.x(), chart.bottom()));

  if (interaction_mode_ == PlotInteractionMode::kSelect) {
    for (const PlotSeriesRuntime* runtime_ptr : series_) {
      if (runtime_ptr == nullptr || !runtime_ptr->config.enabled) {
        continue;
      }
      const std::optional<HoverSample> sample = sampleAtX(*runtime_ptr, hover_x_);
      if (!sample.has_value()) {
        continue;
      }
      const QPointF hit =
          mapToChart(sample->sample_x, sample->y, range);
      QPen hpen(runtime_ptr->config.color, 1, Qt::DotLine);
      hpen.setCosmetic(true);
      painter.setPen(hpen);
      painter.drawLine(QPointF(chart.left(), hit.y()), QPointF(chart.right(), hit.y()));
    }
  }
}

void PlotChartWidget::drawInspection(QPainter& painter, const AxisRange& range,
                                     const QRect& chart) const {
  if (interaction_mode_ != PlotInteractionMode::kInspect) {
    return;
  }

  auto drawMarker = [&](const PlotInspectPoint& point) {
    if (!point.valid) {
      return;
    }
    const QPointF mapped = mapToChart(point.x, point.y, range);
    painter.setPen(QPen(point.color, 2));
    painter.setBrush(Qt::white);
    painter.drawEllipse(mapped, 5.0, 5.0);
    painter.setPen(QPen(point.color, 1, Qt::DashLine));
    painter.drawLine(QPointF(mapped.x(), chart.top()),
                     QPointF(mapped.x(), chart.bottom()));
    painter.drawLine(QPointF(chart.left(), mapped.y()),
                     QPointF(chart.right(), mapped.y()));
  };

  drawMarker(inspect_a_);
  drawMarker(inspect_b_);

  if (inspect_a_.valid && inspect_b_.valid) {
    const QPointF a = mapToChart(inspect_a_.x, inspect_a_.y, range);
    const QPointF b = mapToChart(inspect_b_.x, inspect_b_.y, range);
    QPen pen(QColor(78, 152, 226), 2, Qt::DashLine);
    pen.setCosmetic(true);
    painter.setPen(pen);
    painter.drawLine(a, b);

    QStringList lines;
    lines << tr("Point 1: (%1, %2)")
                 .arg(formatXAxisValue(inspect_a_.x))
                 .arg(QString::number(inspect_a_.y, 'g', 4));
    lines << tr("Point 2: (%1, %2)")
                 .arg(formatXAxisValue(inspect_b_.x))
                 .arg(QString::number(inspect_b_.y, 'g', 4));
    lines << formatDeltaXValue(inspect_b_.x - inspect_a_.x);
    lines << tr("Δy: %1").arg(QString::number(inspect_b_.y - inspect_a_.y, 'g', 4));

    QFontMetrics fm(font());
    int max_w = 0;
    for (const QString& line : lines) {
      max_w = std::max(max_w, fm.horizontalAdvance(line));
    }
    const int box_w = max_w + 20;
    const int box_h = lines.size() * (fm.height() + 4) + 12;
    const QRect box(chart.left() + 8, chart.bottom() - box_h - 8, box_w, box_h);
    painter.setPen(QColor(180, 180, 180));
    painter.setBrush(QColor(255, 255, 255, 240));
    painter.drawRoundedRect(box, 4, 4);
    int y = box.top() + 8;
    painter.setPen(QColor(40, 40, 40));
    for (const QString& line : lines) {
      painter.drawText(QRect(box.left() + 10, y, box.width() - 20, fm.height() + 4),
                       Qt::AlignLeft | Qt::AlignVCenter, line);
      y += fm.height() + 4;
    }
  }
}

void PlotChartWidget::drawTooltip(QPainter& painter) const {
  if (interaction_mode_ == PlotInteractionMode::kInspect) {
    return;
  }
  if (!hover_active_ || tooltip_rows_.isEmpty()) {
    return;
  }

  QFontMetrics fm(font());
  int max_label = 0;
  int max_value = 0;
  for (const TooltipRow& row : tooltip_rows_) {
    max_label = std::max(max_label, fm.horizontalAdvance(row.label));
    max_value = std::max(max_value, fm.horizontalAdvance(row.value_text));
  }

  const int row_h = fm.height() + 4;
  const int width = max_label + max_value + 40;
  const int height = tooltip_rows_.size() * row_h + 12;
  int left = hover_pos_.x() + 16;
  int top = hover_pos_.y() - height - 8;
  if (left + width > this->width() - 8) {
    left = hover_pos_.x() - width - 16;
  }
  if (top < 8) {
    top = hover_pos_.y() + 16;
  }

  const QRect box(left, top, width, height);
  painter.setPen(QColor(180, 180, 180));
  painter.setBrush(QColor(255, 255, 255, 245));
  painter.drawRoundedRect(box, 4, 4);

  int y = box.top() + 8;
  for (int i = 0; i < tooltip_rows_.size(); ++i) {
    const TooltipRow& row = tooltip_rows_.at(i);
    if (i == 0) {
      painter.setPen(QColor(60, 60, 60));
      painter.drawText(QRect(box.left() + 10, y, box.width() - 20, row_h),
                       Qt::AlignLeft | Qt::AlignVCenter, row.label);
      y += row_h;
      continue;
    }

    painter.fillRect(QRect(box.left() + 10, y + row_h / 2 - 4, 8, 8), row.color);
    painter.setPen(row.latched ? QColor(120, 120, 120) : QColor(40, 40, 40));
    painter.drawText(QRect(box.left() + 22, y, max_label, row_h),
                     Qt::AlignLeft | Qt::AlignVCenter, row.label);
    painter.drawText(QRect(box.right() - max_value - 10, y, max_value, row_h),
                     Qt::AlignRight | Qt::AlignVCenter, row.value_text);
    y += row_h;
  }
}

void PlotChartWidget::paintEvent(QPaintEvent* /*event*/) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);

  const AxisRange range = effectiveAxisRange();
  const QRect chart = chartRect();

  painter.fillRect(rect(), palette().color(QPalette::Window));
  drawGrid(painter, range, chart);
  drawSeries(painter, range, chart);
  drawPlaybackLine(painter, range, chart);
  drawHoverCrosshair(painter, range, chart);
  drawInspection(painter, range, chart);

  if (interaction_mode_ == PlotInteractionMode::kZoom && !zoom_rect_.isNull()) {
    painter.setPen(QPen(QColor(78, 152, 226), 1, Qt::DashLine));
    painter.setBrush(QColor(78, 152, 226, 40));
    painter.drawRect(zoom_rect_);
  }

  const QRect add_button = addSeriesButtonRect();
  painter.setPen(QColor(210, 210, 210));
  painter.setBrush(Qt::white);
  painter.drawRoundedRect(add_button, 4, 4);
  painter.fillRect(QRect(add_button.left() + 10, add_button.top() + 10, 8, 8),
                   QColor(QStringLiteral("#4e98e2")));
  painter.setPen(QColor(60, 60, 60));
  painter.drawText(add_button.adjusted(24, 0, -8, 0), Qt::AlignVCenter | Qt::AlignLeft,
                   tr("Click to add a series"));

  drawTooltip(painter);
}

void PlotChartWidget::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton &&
      addSeriesButtonRect().contains(event->pos())) {
    emit addSeriesRequested();
    return;
  }

  const AxisRange range = effectiveAxisRange();
  const QRect chart = chartRect();
  const bool in_chart = chart.contains(event->pos());

  if (event->button() == Qt::MiddleButton && in_chart) {
    beginPanGesture(event->pos(), true);
    event->accept();
    return;
  }

  if (event->button() == Qt::LeftButton && in_chart) {
    if (interaction_mode_ == PlotInteractionMode::kInspect) {
      const std::optional<PlotInspectPoint> picked =
          pickInspectPoint(event->pos(), range);
      if (!picked.has_value()) {
        return;
      }
      if (!inspect_a_.valid) {
        inspect_a_ = *picked;
      } else if (!inspect_b_.valid) {
        inspect_b_ = *picked;
      } else {
        inspect_a_ = *picked;
        inspect_b_ = PlotInspectPoint{};
      }
      update();
      event->accept();
      return;
    }
    if (interaction_mode_ == PlotInteractionMode::kSelect) {
      updateHoverFromMouse(event->pos());
      beginPanGesture(event->pos(), false);
      event->accept();
      return;
    }
    if (interaction_mode_ == PlotInteractionMode::kPan) {
      beginPanGesture(event->pos(), true);
      event->accept();
      return;
    }
    if (interaction_mode_ == PlotInteractionMode::kZoom) {
      dragging_ = true;
      pan_pending_ = false;
      drag_start_ = event->pos();
      zoom_rect_ = QRect(drag_start_, drag_start_);
      event->accept();
      return;
    }
  }
  QWidget::mousePressEvent(event);
}

void PlotChartWidget::mouseMoveEvent(QMouseEvent* event) {
  const QRect chart = chartRect();
  if (chart.contains(event->pos()) &&
      interaction_mode_ != PlotInteractionMode::kZoom && !dragging_ &&
      !pan_pending_) {
    updateHoverFromMouse(event->pos());
  } else if (!dragging_ && !pan_pending_) {
    clearHover();
  }

  if (interaction_mode_ == PlotInteractionMode::kPan ||
      interaction_mode_ == PlotInteractionMode::kSelect ||
      (dragging_ && event->buttons().testFlag(Qt::MiddleButton))) {
    updatePanGesture(event->pos());
    if (dragging_ || pan_pending_) {
      event->accept();
      return;
    }
  }

  if (!dragging_) {
    return;
  }

  if (interaction_mode_ == PlotInteractionMode::kZoom) {
    zoom_rect_ = QRect(drag_start_, event->pos()).normalized();
    update();
  }
}

void PlotChartWidget::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() == Qt::MiddleButton && (dragging_ || pan_pending_)) {
    finishPanGesture(true);
    event->accept();
    return;
  }

  if (event->button() == Qt::LeftButton &&
      interaction_mode_ == PlotInteractionMode::kSelect && pan_pending_ &&
      !exceedsDragThreshold(event->pos()) &&
      x_axis_mode_ == PlotXAxisMode::kTimestamp && hover_active_) {
    pan_pending_ = false;
    emit seekRequested(hover_x_);
    event->accept();
    return;
  }

  if (event->button() == Qt::LeftButton && (dragging_ || pan_pending_)) {
    if (interaction_mode_ == PlotInteractionMode::kPan ||
        interaction_mode_ == PlotInteractionMode::kSelect) {
      finishPanGesture(true);
      event->accept();
      return;
    }
  }

  if (event->button() != Qt::LeftButton || !dragging_) {
    QWidget::mouseReleaseEvent(event);
    return;
  }

  dragging_ = false;
  if (interaction_mode_ == PlotInteractionMode::kZoom) {
    const QRect chart = chartRect();
    const QRect selection = zoom_rect_.normalized().intersected(chart);
    zoom_rect_ = QRect();
    if (selection.width() >= 4 && selection.height() >= 4) {
      applyZoomRect(selection, effectiveAxisRange(), chart);
      emitViewRangeIfNeeded();
    }
    update();
  }
}

void PlotChartWidget::enterEvent(QEnterEvent* event) {
  setFocus(Qt::MouseFocusReason);
  QWidget::enterEvent(event);
}

void PlotChartWidget::leaveEvent(QEvent* /*event*/) {
  clearHover();
}

void PlotChartWidget::applyZoomRect(const QRect& selection,
                                    const AxisRange& range,
                                    const QRect& chart) {
  const double dx = range.max_x - range.min_x;
  const double dy = range.max_y - range.min_y;
  const double left_ratio =
      chart.width() <= 0
          ? 0.0
          : static_cast<double>(selection.left() - chart.left()) / chart.width();
  const double right_ratio =
      chart.width() <= 0
          ? 1.0
          : static_cast<double>(selection.right() - chart.left()) / chart.width();
  const double bottom_ratio =
      chart.height() <= 0
          ? 0.0
          : static_cast<double>(chart.bottom() - selection.bottom()) /
                chart.height();
  const double top_ratio =
      chart.height() <= 0
          ? 1.0
          : static_cast<double>(chart.bottom() - selection.top()) / chart.height();
  view_min_x_ = range.min_x + dx * left_ratio;
  view_max_x_ = range.min_x + dx * right_ratio;
  view_min_y_ = range.min_y + dy * bottom_ratio;
  view_max_y_ = range.min_y + dy * top_ratio;
  view_override_active_ = true;
}

void PlotChartWidget::applyWheelZoom(const QPointF& position, int angle_delta,
                                     Qt::KeyboardModifiers modifiers) {
  if (!chartRect().contains(position.toPoint())) {
    return;
  }

  const bool shift = modifiers.testFlag(Qt::ShiftModifier);
  const bool alt = modifiers.testFlag(Qt::AltModifier);
  const QPoint pos = position.toPoint();
  const QRect chart = chartRect();

  bool zoom_x = true;
  bool zoom_y = false;
  if (pos.x() < chart.left()) {
    zoom_x = false;
    zoom_y = true;
  } else if (pos.y() > chart.bottom()) {
    zoom_x = true;
    zoom_y = false;
  } else if (shift && alt) {
    zoom_x = true;
    zoom_y = true;
  } else if (shift) {
    zoom_x = false;
    zoom_y = true;
  } else if (alt) {
    zoom_x = true;
    zoom_y = false;
  } else if (x_axis_mode_ == PlotXAxisMode::kTimestamp ||
             x_axis_mode_ == PlotXAxisMode::kIndex) {
    zoom_x = true;
    zoom_y = false;
  }

  AxisRange range = effectiveAxisRange();
  const double factor = angle_delta > 0 ? 0.85 : 1.15;
  const double x_ratio =
      chart.width() <= 0 ? 0.5 : (position.x() - chart.left()) / chart.width();
  const double y_ratio =
      chart.height() <= 0
          ? 0.5
          : (chart.bottom() - position.y()) / chart.height();

  const double center_x = range.min_x + (range.max_x - range.min_x) * x_ratio;
  const double center_y = range.min_y + (range.max_y - range.min_y) * y_ratio;
  const double half_dx = (range.max_x - range.min_x) * 0.5;
  const double half_dy = (range.max_y - range.min_y) * 0.5;

  if (zoom_x) {
    const double scaled = half_dx * factor;
    view_min_x_ = center_x - scaled;
    view_max_x_ = center_x + scaled;
  } else {
    view_min_x_ = range.min_x;
    view_max_x_ = range.max_x;
  }

  if (zoom_y) {
    const double scaled = half_dy * factor;
    view_min_y_ = center_y - scaled;
    view_max_y_ = center_y + scaled;
  } else {
    view_min_y_ = range.min_y;
    view_max_y_ = range.max_y;
  }

  view_override_active_ = true;
  emitViewRangeIfNeeded();
  update();
}

void PlotChartWidget::handleWheelZoomAt(const QPointF& local_position,
                                        int angle_delta,
                                        Qt::KeyboardModifiers modifiers) {
  applyWheelZoom(local_position, angle_delta, modifiers);
}

void PlotChartWidget::wheelEvent(QWheelEvent* event) {
  handleWheelZoomAt(event->position(), event->angleDelta().y(), event->modifiers());
  event->accept();
}

void PlotChartWidget::mouseDoubleClickEvent(QMouseEvent* event) {
  if (chartRect().contains(event->pos())) {
    resetView();
    event->accept();
    return;
  }
  QWidget::mouseDoubleClickEvent(event);
}

void PlotChartWidget::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  emit geometryChanged();
  update();
}

void PlotChartWidget::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_R) {
    resetView();
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_Z) {
    emit zoomToolToggleRequested();
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_Escape) {
    if (interaction_mode_ == PlotInteractionMode::kInspect) {
      if (inspect_a_.valid || inspect_b_.valid) {
        clearInspection();
      } else {
        emit inspectEscapeRequested();
      }
      event->accept();
      return;
    }
    dragging_ = false;
    zoom_rect_ = QRect();
    update();
    event->accept();
    return;
  }
  QWidget::keyPressEvent(event);
}

}  // namespace plot
}  // namespace autoviz
