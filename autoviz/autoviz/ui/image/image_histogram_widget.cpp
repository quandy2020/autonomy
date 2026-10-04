/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/image/image_histogram_widget.hpp"

#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>

#include <algorithm>
#include <cmath>

namespace autoviz {
namespace image {

ImageHistogramWidget::ImageHistogramWidget(QWidget* parent) : QWidget(parent) {
  setMinimumHeight(44);
  setMaximumHeight(56);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  setMouseTracking(true);
  setAttribute(Qt::WA_TranslucentBackground, true);
  setToolTip(tr("Drag handles to set colormap Value min / max"));
}

void ImageHistogramWidget::setHistogram(const ImageHistogram& histogram) {
  histogram_ = histogram;
  update();
}

void ImageHistogramWidget::setRange(double min_value, double max_value) {
  range_min_ = std::min(min_value, max_value);
  range_max_ = std::max(min_value, max_value);
  update();
}

int ImageHistogramWidget::valueToX(double value) const {
  const double t = std::clamp(value, 0.0, 255.0) / 255.0;
  return static_cast<int>(std::lround(t * (width() - 1)));
}

double ImageHistogramWidget::xToValue(int x) const {
  if (width() <= 1) {
    return 0.0;
  }
  return std::clamp(static_cast<double>(x) / (width() - 1) * 255.0, 0.0, 255.0);
}

ImageHistogramWidget::DragTarget ImageHistogramWidget::hitTest(
    const QPoint& pos) const {
  const int x_min = valueToX(range_min_);
  const int x_max = valueToX(range_max_);
  if (std::abs(pos.x() - x_min) <= 5) {
    return DragTarget::kMin;
  }
  if (std::abs(pos.x() - x_max) <= 5) {
    return DragTarget::kMax;
  }
  if (pos.x() > x_min && pos.x() < x_max) {
    return DragTarget::kBoth;
  }
  return DragTarget::kNone;
}

void ImageHistogramWidget::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), QColor(255, 255, 255, 28));

  int peak = 1;
  for (int count : histogram_.bins) {
    peak = std::max(peak, count);
  }

  const int w = width();
  const int h = height();
  for (int i = 0; i < 256; ++i) {
    const int x0 = valueToX(i);
    const int x1 = valueToX(i + 1);
    const int bar_w = std::max(1, x1 - x0);
    const int bar_h = static_cast<int>(
        std::lround(static_cast<double>(histogram_.bins[static_cast<size_t>(i)]) /
                    peak * (h - 4)));
    painter.fillRect(x0, h - bar_h, bar_w, bar_h, QColor(8, 145, 178, 140));
  }

  const int x_min = valueToX(range_min_);
  const int x_max = valueToX(range_max_);
  painter.fillRect(0, 0, x_min, h, QColor(15, 23, 42, 28));
  painter.fillRect(x_max, 0, w - x_max, h, QColor(15, 23, 42, 28));
  painter.setPen(QPen(QColor(8, 145, 178), 2));
  painter.drawLine(x_min, 0, x_min, h);
  painter.drawLine(x_max, 0, x_max, h);

  painter.setPen(QColor(30, 41, 59, 210));
  painter.drawText(rect().adjusted(6, 2, -6, -2),
                   Qt::AlignLeft | Qt::AlignTop,
                   QStringLiteral("[%1 – %2]")
                       .arg(range_min_, 0, 'f', 0)
                       .arg(range_max_, 0, 'f', 0));
}

void ImageHistogramWidget::mousePressEvent(QMouseEvent* event) {
  if (event->button() != Qt::LeftButton) {
    return;
  }
  drag_ = hitTest(event->pos());
  if (drag_ == DragTarget::kNone) {
    // Click outside → move nearest handle.
    const double value = xToValue(event->pos().x());
    if (std::abs(value - range_min_) <= std::abs(value - range_max_)) {
      range_min_ = value;
      drag_ = DragTarget::kMin;
    } else {
      range_max_ = value;
      drag_ = DragTarget::kMax;
    }
    if (range_min_ > range_max_) {
      std::swap(range_min_, range_max_);
      drag_ = (drag_ == DragTarget::kMin) ? DragTarget::kMax : DragTarget::kMin;
    }
  }
  drag_anchor_x_ = event->pos().x();
  drag_anchor_min_ = range_min_;
  drag_anchor_max_ = range_max_;
  update();
  event->accept();
}

void ImageHistogramWidget::mouseMoveEvent(QMouseEvent* event) {
  if (drag_ == DragTarget::kNone) {
    return;
  }
  if (drag_ == DragTarget::kMin) {
    range_min_ = std::min(xToValue(event->pos().x()), range_max_);
  } else if (drag_ == DragTarget::kMax) {
    range_max_ = std::max(xToValue(event->pos().x()), range_min_);
  } else if (drag_ == DragTarget::kBoth) {
    const double delta = xToValue(event->pos().x()) - xToValue(drag_anchor_x_);
    double new_min = drag_anchor_min_ + delta;
    double new_max = drag_anchor_max_ + delta;
    if (new_min < 0.0) {
      new_max -= new_min;
      new_min = 0.0;
    }
    if (new_max > 255.0) {
      new_min -= (new_max - 255.0);
      new_max = 255.0;
    }
    range_min_ = std::clamp(new_min, 0.0, 255.0);
    range_max_ = std::clamp(new_max, 0.0, 255.0);
  }
  update();
  event->accept();
}

void ImageHistogramWidget::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton && drag_ != DragTarget::kNone) {
    drag_ = DragTarget::kNone;
    emit rangeChanged(range_min_, range_max_);
  }
  QWidget::mouseReleaseEvent(event);
}

}  // namespace image
}  // namespace autoviz
