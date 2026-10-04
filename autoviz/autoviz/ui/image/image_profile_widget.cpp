/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/image/image_profile_widget.hpp"

#include <QPainter>
#include <QPaintEvent>

#include <algorithm>
#include <cmath>

namespace autoviz {
namespace image {

ImageProfileWidget::ImageProfileWidget(QWidget* parent) : QWidget(parent) {
  setMinimumHeight(40);
  setMaximumHeight(52);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  setAttribute(Qt::WA_TranslucentBackground, true);
  setToolTip(tr("Line profile (luminance along Profile segment)"));
}

void ImageProfileWidget::setSamples(const QVector<double>& samples) {
  samples_ = samples;
  update();
}

void ImageProfileWidget::clear() {
  samples_.clear();
  update();
}

void ImageProfileWidget::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), QColor(255, 255, 255, 28));
  painter.setPen(QColor(100, 116, 139));
  painter.drawText(rect().adjusted(6, 2, -6, -2), Qt::AlignLeft | Qt::AlignTop,
                   tr("Profile"));

  if (samples_.size() < 2) {
    painter.setPen(QColor(148, 163, 184));
    painter.drawText(rect(), Qt::AlignCenter, tr("Draw a Profile line"));
    return;
  }

  double lo = samples_.front();
  double hi = samples_.front();
  for (double v : samples_) {
    lo = std::min(lo, v);
    hi = std::max(hi, v);
  }
  if (hi - lo < 1e-6) {
    hi = lo + 1.0;
  }

  const QRect plot = rect().adjusted(6, 16, -6, -6);
  QPolygonF poly;
  poly.reserve(samples_.size());
  for (int i = 0; i < samples_.size(); ++i) {
    const double t = static_cast<double>(i) / (samples_.size() - 1);
    const double n = (samples_.at(i) - lo) / (hi - lo);
    poly << QPointF(plot.left() + t * plot.width(),
                    plot.bottom() - n * plot.height());
  }
  painter.setPen(QPen(QColor(8, 145, 178), 1.5));
  painter.drawPolyline(poly);
  painter.setPen(QColor(30, 41, 59, 200));
  painter.drawText(plot.adjusted(0, 0, 0, 0), Qt::AlignRight | Qt::AlignTop,
                   QStringLiteral("%1–%2").arg(lo, 0, 'f', 0).arg(hi, 0, 'f', 0));
}

}  // namespace image
}  // namespace autoviz
