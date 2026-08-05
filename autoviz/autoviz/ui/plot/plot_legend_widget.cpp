/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/plot_legend_widget.hpp"

#include <QPainter>

#include "autoviz/ui/plot/plot_types.hpp"

namespace autoviz {
namespace plot {

PlotLegendWidget::PlotLegendWidget(QWidget* parent) : QFrame(parent) {
  setFrameShape(QFrame::NoFrame);
  setAutoFillBackground(true);
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

QSize PlotLegendWidget::sizeHint() const {
  int visible = 0;
  for (const PlotSeriesRuntime* runtime : series_) {
    if (runtime != nullptr && runtime->config.enabled) {
      ++visible;
    }
  }
  if (visible == 0) {
    return QSize(120, 28);
  }
  const int row_h = show_values_ ? 34 : 20;
  return QSize(show_values_ ? 220 : 180, 8 + visible * row_h);
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
    if (runtime_ptr == nullptr || !runtime_ptr->config.enabled) {
      continue;
    }
    any = true;
    const PlotSeriesConfig& config = runtime_ptr->config;
    const QString label =
        config.label.isEmpty() ? config.field_path : config.label;

    painter.fillRect(QRect(8, y + 4, 10, 10), config.color);
    painter.setPen(QColor(60, 60, 60));
    painter.drawText(QRect(24, y, width() - 32, 16), Qt::AlignVCenter | Qt::AlignLeft,
                     label);

    if (show_values_) {
      const QString value = valueForLabel(label, config.color);
      if (!value.isEmpty()) {
        painter.setPen(QColor(100, 100, 100));
        painter.drawText(QRect(24, y + 16, width() - 32, 14),
                         Qt::AlignVCenter | Qt::AlignLeft, value);
      }
    }

    y += show_values_ ? 34 : 20;
  }

  if (!any) {
    painter.setPen(QColor(120, 120, 120));
    painter.drawText(rect().adjusted(8, 0, -8, 0), Qt::AlignCenter,
                     tr("No series"));
  }
}

}  // namespace plot
}  // namespace autoviz
