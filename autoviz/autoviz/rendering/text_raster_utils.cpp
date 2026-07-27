/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/text_raster_utils.hpp"

#include <algorithm>

#include <QFont>
#include <QFontMetrics>
#include <QPainter>

namespace autoviz {
namespace rendering {

QImage RasterizeTextLabel(const QString& text, const QColor& color,
                          int pixel_height) {
  if (text.isEmpty()) {
    return QImage();
  }
  const int font_px = std::max(8, pixel_height);
  QFont font;
  font.setPixelSize(font_px);
  const QFontMetrics metrics(font);
  const QRect bounds = metrics.boundingRect(text);
  const int pad = 2;
  QImage image(bounds.width() + pad * 2, bounds.height() + pad * 2,
               QImage::Format_RGBA8888);
  image.fill(Qt::transparent);
  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::TextAntialiasing, true);
  painter.setFont(font);
  painter.setPen(color);
  painter.drawText(pad, pad + metrics.ascent(), text);
  return image;
}

}  // namespace rendering
}  // namespace autoviz
