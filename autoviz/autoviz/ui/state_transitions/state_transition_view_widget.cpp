/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/state_transitions/state_transition_view_widget.hpp"

#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QFontMetrics>

#include <algorithm>
#include <cmath>

namespace autoviz {
namespace state_transitions {
namespace {

constexpr int kAddButtonSize = 18;

QString FormatTimeLabel(double seconds) {
  if (std::abs(seconds) >= 3600.0) {
    return QStringLiteral("%1h").arg(seconds / 3600.0, 0, 'f', 1);
  }
  if (std::abs(seconds) >= 60.0) {
    return QStringLiteral("%1m").arg(seconds / 60.0, 0, 'f', 1);
  }
  return QStringLiteral("%1s").arg(seconds, 0, 'f', 1);
}

}  // namespace

StateTransitionViewWidget::StateTransitionViewWidget(QWidget* parent)
    : QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
  setMinimumHeight(100);
  QFont font = this->font();
  font.setPointSizeF(std::max(8.5, font.pointSizeF() - 0.5));
  setFont(font);
}

void StateTransitionViewWidget::setBoxZoomMode(bool enabled) {
  if (box_zoom_mode_ == enabled) {
    return;
  }
  box_zoom_mode_ = enabled;
  box_zoom_dragging_ = false;
  update();
  emit boxZoomModeChanged(enabled);
}

void StateTransitionViewWidget::setSeries(
    const std::vector<const StateTransitionSeriesRuntime*>& series) {
  series_ = series;
  clampRowScroll();
  update();
}

void StateTransitionViewWidget::setPanelConfig(
    const StateTransitionPanelConfig& config) {
  config_ = config;
  if (!has_custom_view_) {
    const TimeRange range = defaultTimeRange();
    view_min_time_ = range.min_time;
    view_max_time_ = range.max_time;
  }
  update();
}

void StateTransitionViewWidget::setReferenceTimeSec(double time_sec) {
  reference_time_sec_ = time_sec;
  if (!has_custom_view_) {
    const TimeRange range = defaultTimeRange();
    view_min_time_ = range.min_time;
    view_max_time_ = range.max_time;
  }
  update();
}

void StateTransitionViewWidget::resetView() {
  has_custom_view_ = false;
  row_scroll_offset_ = 0;
  const TimeRange range = defaultTimeRange();
  view_min_time_ = range.min_time;
  view_max_time_ = range.max_time;
  update();
}

StateTransitionViewWidget::TimeRange StateTransitionViewWidget::defaultTimeRange()
    const {
  TimeRange range;
  if (config_.x_axis_mode == StateXAxisMode::kFixedRange) {
    range.min_time = config_.fixed_min_time;
    range.max_time = std::max(config_.fixed_max_time, config_.fixed_min_time + 1.0);
    return range;
  }
  range.max_time = reference_time_sec_;
  range.min_time = reference_time_sec_ - std::max(config_.x_window_sec, 1.0);
  return range;
}

StateTransitionViewWidget::TimeRange
StateTransitionViewWidget::effectiveTimeRange() const {
  TimeRange range;
  range.min_time = view_min_time_;
  range.max_time = std::max(view_max_time_, view_min_time_ + 1e-6);
  return range;
}

QRect StateTransitionViewWidget::contentRect() const {
  return QRect(0, headerHeight(), width(),
               std::max(0, height() - headerHeight() - timeAxisHeight()));
}

QRect StateTransitionViewWidget::labelColumnRect() const {
  const QRect content = contentRect();
  return QRect(content.left(), content.top(), labelColumnWidth(), content.height());
}

QRect StateTransitionViewWidget::chartRect() const {
  const QRect content = contentRect();
  return QRect(content.left() + labelColumnWidth(), content.top(),
               std::max(0, content.width() - labelColumnWidth()), content.height());
}

QRect StateTransitionViewWidget::addSeriesButtonRect() const {
  const QRect label = labelColumnRect();
  return QRect(label.left() + 6,
               label.bottom() - kAddButtonSize - 4,
               kAddButtonSize, kAddButtonSize);
}

int StateTransitionViewWidget::visibleRowCount() const {
  const int available = contentRect().height();
  return std::max(1, available / rowHeight());
}

void StateTransitionViewWidget::clampRowScroll() {
  const int max_offset =
      std::max(0, static_cast<int>(series_.size()) - visibleRowCount());
  row_scroll_offset_ = std::clamp(row_scroll_offset_, 0, max_offset);
}

QRect StateTransitionViewWidget::fullRowRect(int visible_row_index) const {
  const QRect content = contentRect();
  return QRect(content.left(), content.top() + visible_row_index * rowHeight(),
               content.width(), rowHeight());
}

QRect StateTransitionViewWidget::chartRowRect(int visible_row_index) const {
  const QRect chart = chartRect();
  return QRect(chart.left(), chart.top() + visible_row_index * rowHeight(),
               chart.width(), rowHeight());
}

double StateTransitionViewWidget::mapTimeToX(double time_sec,
                                             const TimeRange& range,
                                             const QRect& chart) const {
  const double span = range.max_time - range.min_time;
  if (span <= 0.0 || chart.width() <= 0) {
    return chart.left();
  }
  const double ratio = (time_sec - range.min_time) / span;
  return chart.left() + ratio * chart.width();
}

std::optional<double> StateTransitionViewWidget::mapXToTime(
    int x, const TimeRange& range, const QRect& chart) const {
  if (chart.width() <= 0) {
    return std::nullopt;
  }
  const double ratio =
      static_cast<double>(x - chart.left()) / static_cast<double>(chart.width());
  return range.min_time + ratio * (range.max_time - range.min_time);
}

std::size_t StateTransitionViewWidget::lowerSegmentBound(
    const std::deque<StateSegment>& segments, double min_time) const {
  if (segments.empty()) {
    return 0;
  }
  auto it = std::upper_bound(
      segments.begin(), segments.end(), min_time,
      [](double time, const StateSegment& segment) { return time < segment.end_time; });
  if (it == segments.begin()) {
    return 0;
  }
  return static_cast<std::size_t>(std::distance(segments.begin(), it - 1));
}

void StateTransitionViewWidget::drawHeader(QPainter& painter) const {
  const QRect header(0, 0, width(), headerHeight());
  painter.fillRect(header, palette().base());
  painter.setPen(palette().mid().color());
  painter.drawLine(header.bottomLeft(), header.bottomRight());

  QFont header_font = painter.font();
  header_font.setBold(true);
  painter.setFont(header_font);
  painter.setPen(palette().text().color());
  painter.drawText(QRect(8, 0, labelColumnWidth() - 10, headerHeight()),
                   Qt::AlignLeft | Qt::AlignVCenter, tr("Series"));

  QFont hint_font = painter.font();
  hint_font.setBold(false);
  hint_font.setPointSizeF(std::max(8.0, hint_font.pointSizeF() - 0.5));
  painter.setFont(hint_font);
  painter.setPen(palette().mid().color());
  const QRect chart = chartRect();
  painter.drawText(chart.adjusted(8, 0, -8, 0), Qt::AlignLeft | Qt::AlignVCenter,
                   tr("Click seek · Z zoom · Shift+scroll rows"));
}

void StateTransitionViewWidget::drawLabelColumnBackground(QPainter& painter) const {
  const QRect label = labelColumnRect();
  painter.fillRect(label, palette().window());
  painter.setPen(palette().midlight().color());
  painter.drawLine(label.topRight(), label.bottomRight());
}

void StateTransitionViewWidget::drawEmptyState(QPainter& painter,
                                               const QRect& chart) const {
  if (!series_.empty()) {
    return;
  }
  painter.setPen(palette().mid().color());
  QFont hint_font = painter.font();
  hint_font.setItalic(true);
  painter.setFont(hint_font);
  painter.drawText(chart.adjusted(12, 0, -12, 0),
                   Qt::AlignLeft | Qt::AlignVCenter,
                   tr("Drag discrete fields from Channels, or click + to add a series."));
}

void StateTransitionViewWidget::drawTimeAxis(QPainter& painter,
                                             const TimeRange& range,
                                             const QRect& chart) const {
  const QRect axis(chart.left(), height() - timeAxisHeight(), chart.width(),
                   timeAxisHeight());
  painter.fillRect(axis, palette().base());
  painter.setPen(palette().mid().color());
  painter.drawLine(axis.topLeft(), axis.topRight());

  const int tick_count = 5;
  QFontMetrics metrics(painter.font());
  for (int i = 0; i <= tick_count; ++i) {
    const double ratio = static_cast<double>(i) / tick_count;
    const double time = range.min_time + ratio * (range.max_time - range.min_time);
    const int x = static_cast<int>(mapTimeToX(time, range, chart));
    painter.drawLine(x, axis.top(), x, axis.top() + 4);
    const QString label = FormatTimeLabel(time);
    painter.setPen(palette().text().color());
    const int text_x = std::max(chart.left(),
                                x - metrics.horizontalAdvance(label) / 2);
    painter.drawText(text_x, axis.bottom() - 3, label);
    painter.setPen(palette().mid().color());
  }
}

void StateTransitionViewWidget::drawSeriesRows(QPainter& painter,
                                               const TimeRange& range,
                                               const QRect& chart) const {
  const int rows = visibleRowCount();
  for (int visible_index = 0; visible_index < rows; ++visible_index) {
    const int series_index = row_scroll_offset_ + visible_index;
    const QRect row = fullRowRect(visible_index);
    if (visible_index % 2 == 0) {
      painter.fillRect(row, palette().alternateBase());
    }

    if (series_index < 0 ||
        series_index >= static_cast<int>(series_.size()) ||
        series_[series_index] == nullptr) {
      continue;
    }

    const StateTransitionSeriesRuntime& runtime = *series_[series_index];
    const QString series_label =
        runtime.config.label.isEmpty() ? runtime.config.field_path
                                       : runtime.config.label;
    painter.setPen(runtime.config.enabled ? palette().text().color()
                                          : palette().mid().color());
    const QRect label_rect(labelColumnRect().left() + 8, row.top(),
                           labelColumnWidth() - 12, row.height());
    painter.drawText(label_rect, Qt::AlignVCenter | Qt::AlignLeft,
                     QFontMetrics(painter.font()).elidedText(
                         series_label, Qt::ElideRight, label_rect.width()));

    if (!runtime.config.enabled) {
      continue;
    }

    const QRect bar_row = chartRowRect(visible_index);
    const QRect bar_rect = bar_row.adjusted(0, 3, 0, -3);
    const std::size_t start_index =
        lowerSegmentBound(runtime.segments, range.min_time);
    for (std::size_t i = start_index; i < runtime.segments.size(); ++i) {
      const StateSegment& segment = runtime.segments[i];
      if (segment.start_time > range.max_time) {
        break;
      }
      const double clip_start = std::max(segment.start_time, range.min_time);
      const double clip_end = std::min(segment.end_time, range.max_time);
      if (clip_end <= clip_start) {
        continue;
      }
      const int x0 =
          static_cast<int>(mapTimeToX(clip_start, range, chart));
      const int x1 =
          static_cast<int>(mapTimeToX(clip_end, range, chart));
      const int width = std::max(2, x1 - x0);
      QRect segment_rect(x0, bar_rect.top(), width, bar_rect.height());
      painter.fillRect(segment_rect, segment.color);
      painter.setPen(segment.color.darker(115));
      painter.drawRect(segment_rect);
      if (width > 32) {
        painter.setPen(Qt::black);
        painter.drawText(segment_rect.adjusted(3, 0, -3, 0),
                         Qt::AlignVCenter | Qt::AlignLeft,
                         QFontMetrics(painter.font()).elidedText(
                             segment.label, Qt::ElideRight, segment_rect.width() - 6));
      }
    }
  }
}

void StateTransitionViewWidget::drawAddSeriesButton(QPainter& painter) const {
  const QRect add_button = addSeriesButtonRect();
  painter.setPen(box_zoom_mode_ ? palette().mid().color()
                                : palette().dark().color());
  painter.setBrush(palette().button());
  painter.drawRoundedRect(add_button, 3, 3);
  painter.drawLine(add_button.center().x(), add_button.top() + 5,
                   add_button.center().x(), add_button.bottom() - 5);
  painter.drawLine(add_button.left() + 5, add_button.center().y(),
                   add_button.right() - 5, add_button.center().y());
}

void StateTransitionViewWidget::drawBoxZoomOverlay(QPainter& painter) const {
  if (!box_zoom_dragging_) {
    return;
  }
  QRect rect(box_zoom_start_, box_zoom_end_);
  rect = rect.normalized();
  painter.fillRect(rect, QColor(80, 140, 220, 40));
  painter.setPen(QPen(QColor(80, 140, 220), 1, Qt::DashLine));
  painter.drawRect(rect);
}

void StateTransitionViewWidget::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, false);
  painter.fillRect(rect(), palette().window());

  drawHeader(painter);

  const TimeRange range = effectiveTimeRange();
  const QRect chart = chartRect();
  drawLabelColumnBackground(painter);
  painter.fillRect(chart, palette().base());
  drawSeriesRows(painter, range, chart);
  drawEmptyState(painter, chart);
  drawTimeAxis(painter, range, chart);
  drawAddSeriesButton(painter);
  drawBoxZoomOverlay(painter);

  if (box_zoom_mode_) {
    painter.setPen(QColor(80, 140, 220));
    QFont hint_font = painter.font();
    hint_font.setPointSizeF(std::max(8.0, hint_font.pointSizeF() - 0.5));
    painter.setFont(hint_font);
    painter.drawText(chart.adjusted(8, 4, -8, -4), Qt::AlignTop | Qt::AlignLeft,
                     tr("Box zoom — drag to select range, Z to exit"));
  }
}

void StateTransitionViewWidget::mousePressEvent(QMouseEvent* event) {
  if (event == nullptr) {
    return;
  }
  if (event->button() == Qt::LeftButton &&
      addSeriesButtonRect().contains(event->pos())) {
    emit addSeriesRequested();
    return;
  }
  if (event->button() == Qt::LeftButton && box_zoom_mode_) {
    box_zoom_dragging_ = true;
    box_zoom_start_ = event->pos();
    box_zoom_end_ = event->pos();
    update();
    return;
  }
  if (event->button() == Qt::LeftButton) {
    panning_ = true;
    pan_anchor_ = event->pos();
    pan_anchor_min_time_ = view_min_time_;
    pan_anchor_row_scroll_ = row_scroll_offset_;
  }
}

void StateTransitionViewWidget::mouseMoveEvent(QMouseEvent* event) {
  if (event == nullptr) {
    return;
  }
  if (box_zoom_dragging_) {
    box_zoom_end_ = event->pos();
    update();
    return;
  }
  if (!panning_) {
    return;
  }
  const QRect chart = chartRect();
  const double span = view_max_time_ - view_min_time_;
  if (chart.width() > 0 && span > 0.0) {
    const double delta_pixels = event->pos().x() - pan_anchor_.x();
    const double delta_time = -delta_pixels * span / chart.width();
    view_min_time_ = pan_anchor_min_time_ + delta_time;
    view_max_time_ = view_min_time_ + span;
    has_custom_view_ = true;
  }
  const int delta_rows = (event->pos().y() - pan_anchor_.y()) / rowHeight();
  row_scroll_offset_ = pan_anchor_row_scroll_ - delta_rows;
  clampRowScroll();
  update();
}

void StateTransitionViewWidget::mouseReleaseEvent(QMouseEvent* event) {
  if (event == nullptr) {
    return;
  }
  if (box_zoom_dragging_ && event->button() == Qt::LeftButton) {
    box_zoom_dragging_ = false;
    const QRect chart = chartRect();
    QRect zoom_rect(box_zoom_start_, event->pos());
    zoom_rect = zoom_rect.normalized();
    if (zoom_rect.width() > 8) {
      if (const std::optional<double> t0 = mapXToTime(zoom_rect.left(), effectiveTimeRange(), chart)) {
        if (const std::optional<double> t1 = mapXToTime(zoom_rect.right(), effectiveTimeRange(), chart)) {
          view_min_time_ = std::min(*t0, *t1);
          view_max_time_ = std::max(*t0, *t1);
          has_custom_view_ = true;
        }
      }
    }
    setBoxZoomMode(false);
    update();
    return;
  }
  if (event->button() == Qt::LeftButton && panning_) {
    const QPoint delta = event->pos() - pan_anchor_;
    if (delta.manhattanLength() <= 3) {
      if (const std::optional<double> time =
              mapXToTime(event->pos().x(), effectiveTimeRange(), chartRect())) {
        emit seekRequested(*time);
      }
    }
  }
  panning_ = false;
}

void StateTransitionViewWidget::wheelEvent(QWheelEvent* event) {
  if (event == nullptr) {
    return;
  }
  const int delta = event->angleDelta().y();
  if (delta == 0) {
    return;
  }
  if (event->modifiers().testFlag(Qt::ShiftModifier)) {
    row_scroll_offset_ += delta > 0 ? -1 : 1;
    clampRowScroll();
    update();
    event->accept();
    return;
  }

  const QRect chart = chartRect();
  const double span = view_max_time_ - view_min_time_;
  if (span <= 0.0) {
    return;
  }
  const double anchor_ratio =
      chart.width() > 0
          ? static_cast<double>(event->position().x() - chart.left()) / chart.width()
          : 0.5;
  const double anchor_time = view_min_time_ + anchor_ratio * span;
  const double zoom_factor = delta > 0 ? 0.85 : 1.15;
  const double new_span = std::clamp(span * zoom_factor, 0.5, 86400.0);
  view_min_time_ = anchor_time - anchor_ratio * new_span;
  view_max_time_ = view_min_time_ + new_span;
  has_custom_view_ = true;
  update();
  event->accept();
}

void StateTransitionViewWidget::keyPressEvent(QKeyEvent* event) {
  if (event == nullptr) {
    return;
  }
  if (event->key() == Qt::Key_Z) {
    setBoxZoomMode(!box_zoom_mode_);
    event->accept();
    return;
  }
  QWidget::keyPressEvent(event);
}

void StateTransitionViewWidget::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  clampRowScroll();
}

}  // namespace state_transitions
}  // namespace autoviz
