/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/image/image_processing.hpp"

#include <algorithm>
#include <cmath>

#include <QColor>
#include <QPainter>
#include <QRgb>

namespace autoviz {
namespace image {
namespace {

QColor TurboColor(double t) {
  t = std::clamp(t, 0.0, 1.0);
  const double r = std::clamp(1.5 - std::abs(4.0 * t - 3.0), 0.0, 1.0);
  const double g = std::clamp(1.5 - std::abs(4.0 * t - 2.0), 0.0, 1.0);
  const double b = std::clamp(1.5 - std::abs(4.0 * t - 1.0), 0.0, 1.0);
  return QColor(static_cast<int>(r * 255.0), static_cast<int>(g * 255.0),
                static_cast<int>(b * 255.0));
}

QColor RainbowColor(double t) {
  t = std::clamp(t, 0.0, 1.0);
  const double hue = (1.0 - t) * 240.0;
  QColor color;
  color.setHsvF(hue / 360.0, 1.0, 1.0);
  return color;
}

double NormalizeValue(int value, double min_value, double max_value) {
  if (max_value <= min_value) {
    return 0.0;
  }
  return (static_cast<double>(value) - min_value) / (max_value - min_value);
}

QImage applyMaskAlpha(const QImage& overlay, ImagePixelAlpha pixel_alpha) {
  if (pixel_alpha != ImagePixelAlpha::kWhiteTransparent || overlay.isNull()) {
    return overlay;
  }
  QImage masked = overlay.convertToFormat(QImage::Format_ARGB32);
  for (int y = 0; y < masked.height(); ++y) {
    auto* line = reinterpret_cast<QRgb*>(masked.scanLine(y));
    for (int x = 0; x < masked.width(); ++x) {
      const QRgb pixel = line[x];
      const int r = qRed(pixel);
      const int g = qGreen(pixel);
      const int b = qBlue(pixel);
      if (r > 240 && g > 240 && b > 240) {
        line[x] = qRgba(r, g, b, 0);
      }
    }
  }
  return masked;
}

}  // namespace

QImage applyDisplayTransform(const QImage& source, bool flip_horizontal,
                             bool flip_vertical, ImageRotation rotation) {
  if (source.isNull()) {
    return {};
  }
  if (!flip_horizontal && !flip_vertical && rotation == ImageRotation::k0) {
    return source;
  }
  QTransform transform;
  if (flip_horizontal) {
    transform.scale(-1.0, 1.0);
    transform.translate(-source.width(), 0.0);
  }
  if (flip_vertical) {
    QTransform flip_y;
    flip_y.scale(1.0, -1.0);
    flip_y.translate(0.0, -source.height());
    transform = transform * flip_y;
  }
  switch (rotation) {
    case ImageRotation::k90:
      transform.rotate(90.0);
      break;
    case ImageRotation::k180:
      transform.rotate(180.0);
      break;
    case ImageRotation::k270:
      transform.rotate(270.0);
      break;
    case ImageRotation::k0:
    default:
      break;
  }
  // QImage::transformed() on packed 24-bit RGB leaves black tiles on some
  // Qt/raster backends. Convert to a 32-bit format first.
  const QImage rgb32 =
      (source.format() == QImage::Format_RGB32 ||
       source.format() == QImage::Format_ARGB32 ||
       source.format() == QImage::Format_ARGB32_Premultiplied)
          ? source
          : source.convertToFormat(QImage::Format_RGB32);
  return rgb32.transformed(transform, Qt::SmoothTransformation);
}

QImage applyColorMode(const QImage& source, ImageColorMode mode,
                      double min_value, double max_value) {
  if (source.isNull() || mode == ImageColorMode::kOff) {
    return source;
  }
  QImage gray = source;
  if (gray.format() != QImage::Format_Grayscale8) {
    gray = gray.convertToFormat(QImage::Format_Grayscale8);
  }
  QImage colored(gray.size(), QImage::Format_RGB32);
  for (int y = 0; y < gray.height(); ++y) {
    const auto* src = reinterpret_cast<const uchar*>(gray.constScanLine(y));
    auto* dst = reinterpret_cast<QRgb*>(colored.scanLine(y));
    for (int x = 0; x < gray.width(); ++x) {
      const double t = NormalizeValue(src[x], min_value, max_value);
      QColor color;
      switch (mode) {
        case ImageColorMode::kTurbo:
          color = TurboColor(t);
          break;
        case ImageColorMode::kRainbow:
          color = RainbowColor(t);
          break;
        case ImageColorMode::kGrayscale:
        default:
          color = QColor(src[x], src[x], src[x]);
          break;
      }
      dst[x] = color.rgb();
    }
  }
  return colored;
}

QImage compositeOverlay(const QImage& base, const QImage& overlay,
                        const ImageOverlayConfig& config) {
  if (base.isNull()) {
    return overlay;
  }
  if (overlay.isNull() || !config.enabled) {
    return base;
  }
  QImage canvas = base.convertToFormat(QImage::Format_ARGB32);
  QImage layer = overlay;
  if (layer.size() != canvas.size()) {
    layer = layer.scaled(canvas.size(), Qt::IgnoreAspectRatio,
                         Qt::SmoothTransformation);
  }
  layer = applyMaskAlpha(layer, config.pixel_alpha);
  layer = layer.convertToFormat(QImage::Format_ARGB32);

  QPainter painter(&canvas);
  painter.setOpacity(std::clamp(config.opacity, 0.0, 1.0));
  if (config.blend_mode == ImageBlendMode::kAdd) {
    painter.setCompositionMode(QPainter::CompositionMode_Plus);
  } else {
    painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
  }
  painter.drawImage(0, 0, layer);
  painter.end();
  return canvas;
}

}  // namespace image
}  // namespace autoviz
