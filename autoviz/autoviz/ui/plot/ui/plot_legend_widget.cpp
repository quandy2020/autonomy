/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/ui/plot_legend_widget.hpp"

#include <algorithm>

#include <QFontMetrics>
#include <QMouseEvent>
#include <QPainter>

#include "autoviz/ui/plot/field/plot_path_utils.hpp"
#include "autoviz/ui/plot/plot_types.hpp"

namespace autoviz {
namespace plot {

PlotLegendWidget::PlotLegendWidget(QWidget* parent) : QFrame(parent) {
  setFrameShape(QFrame::NoFrame);
  setAutoFillBackground(true);
  setAttribute(Qt::WA_TransparentForMouseEvents, false);
  setCursor(Qt::PointingHandCursor);
  QPalette pal = palette();
  pal.setColor(QPalette::Window, QColor(255, 255, 255, 230));
  setPalette(pal);
  setVisible(false);
}

void PlotLegendWidget::setSeries(
    const std::vector<const PlotSeriesRuntime*>& series) {
  series_ = series;
  updateGeometry();
  update();
}

void PlotLegendWidget::setShowValues(bool show) {
  show_values_ = show;
  updateGeometry();
  update();
}

void PlotLegendWidget::setValueRows(const QVector<PlotValueRow>& rows) {
  value_rows_ = rows;
  update();
}

QSize PlotLegendWidget::preferredSize() const { return sizeHint(); }

int PlotLegendWidget::rowHeight() const { return show_values_ ? 34 : 20; }

QSize PlotLegendWidget::sizeHint() const {
  int visible = 0;
  int max_label_w = 0;
  const QFontMetrics fm(font());
  for (const PlotSeriesRuntime* runtime : series_) {
    if (runtime == nullptr) {
      continue;
    }
    ++visible;
    const QString label = SeriesLegendLabel(
        runtime->config.label, runtime->config.channel, runtime->config.field_path);
    max_label_w = std::max(max_label_w, fm.horizontalAdvance(label));
    for (const PlotValueRow& row : value_rows_) {
      if (row.label == label && row.color == runtime->config.color) {
        max_label_w = std::max(max_label_w, fm.horizontalAdvance(row.value_text));
        break;
      }
    }
  }
  if (visible == 0) {
    return QSize(120, 28);
  }
  const int width = std::max(180, 8 + 10 + 6 + max_label_w + 12);
  return QSize(width, 8 + visible * rowHeight());
}

void PlotLegendWidget::mousePressEvent(QMouseEvent* event) {
  if (event == nullptr || event->button() != Qt::LeftButton) {
    QFrame::mousePressEvent(event);
    return;
  }
  const int y = event->position().toPoint().y() - 8;
  if (y < 0) {
    return;
  }
  const int row = y / rowHeight();
  int visible_index = 0;
  for (int i = 0; i < static_cast<int>(series_.size()); ++i) {
    if (series_[static_cast<std::size_t>(i)] == nullptr) {
      continue;
    }
    if (visible_index == row) {
      emit seriesVisibilityToggled(i);
      event->accept();
      return;
    }
    ++visible_index;
  }
  QFrame::mousePressEvent(event);
}

void PlotLegendWidget::paintEvent(QPaintEvent* /*event*/) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), palette().color(QPalette::Window));

  auto valueForLabel = [&](const QString& label, const QColor& color) -> QString {
    for (const PlotValueRow& row : value_rows_) {
      if (row.label == label && row.color == color) {
        return row.value_text;
      }
    }
    return QString();
  };

  int y = 8;
  bool any = false;
  for (const PlotSeriesRuntime* runtime_ptr : series_) {
    if (runtime_ptr == nullptr) {
      continue;
    }
    any = true;
    const PlotSeriesConfig& config = runtime_ptr->config;
    const QString label =
        SeriesLegendLabel(config.label, config.channel, config.field_path);
    const bool enabled = config.enabled;
    QColor swatch = config.color;
    if (!enabled) {
      swatch.setAlpha(90);
    }
    painter.fillRect(QRect(8, y + 4, 10, 10), swatch);
    painter.setPen(enabled ? QColor(60, 60, 60) : QColor(160, 160, 160));
    painter.drawText(QRect(24, y, width() - 32, 16), Qt::AlignVCenter | Qt::AlignLeft,
                     label);
    if (!enabled) {
      const QFontMetrics fm(painter.font());
      const int text_w = fm.horizontalAdvance(label);
      painter.setPen(QColor(160, 160, 160));
      painter.drawLine(24, y + 8, 24 + text_w, y + 8);
    }

    if (show_values_ && enabled) {
      const QString value = valueForLabel(label, config.color);
      if (!value.isEmpty()) {
        painter.setPen(QColor(100, 100, 100));
        painter.drawText(QRect(24, y + 16, width() - 32, 14),
                         Qt::AlignVCenter | Qt::AlignLeft, value);
      }
    }

    y += rowHeight();
  }

  if (!any) {
    painter.setPen(QColor(120, 120, 120));
    painter.drawText(rect().adjusted(8, 0, -8, 0), Qt::AlignCenter,
                     tr("No series"));
  }
}

}  // namespace plot
}  // namespace autoviz
