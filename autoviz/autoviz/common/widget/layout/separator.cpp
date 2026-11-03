/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/layout/separator.hpp"

#include <QPaintEvent>
#include <QPainter>

namespace autoviz::widget {

Separator::Separator(Qt::Orientation orientation, QWidget* parent)
    : QWidget(parent), orientation_(orientation) {
  setObjectName(QStringLiteral("AutovizSeparator"));
  setProperty("autovizRole", QStringLiteral("separator"));
  setAttribute(Qt::WA_TransparentForMouseEvents);
  SetOrientation(orientation);
}

void Separator::SetOrientation(Qt::Orientation orientation) {
  orientation_ = orientation;
  setSizePolicy(orientation == Qt::Horizontal ? QSizePolicy::Expanding
                                              : QSizePolicy::Fixed,
                orientation == Qt::Horizontal ? QSizePolicy::Fixed
                                              : QSizePolicy::Expanding);
  updateGeometry();
  update();
}

void Separator::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  update();
}

QSize Separator::sizeHint() const {
  return orientation_ == Qt::Horizontal ? QSize(32, 1) : QSize(1, 32);
}

QSize Separator::minimumSizeHint() const { return sizeHint(); }

void Separator::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  QPainter painter(this);
  painter.setPen(QPen(GetThemeTokens(tone_).border, 1));
  if (orientation_ == Qt::Horizontal) {
    const qreal y = (height() - 1) * 0.5;
    painter.drawLine(QPointF(0, y), QPointF(width(), y));
  } else {
    const qreal x = (width() - 1) * 0.5;
    painter.drawLine(QPointF(x, 0), QPointF(x, height()));
  }
}

}  // namespace autoviz::widget
