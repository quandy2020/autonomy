/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/display/watermark.hpp"

#include <QEvent>
#include <QFontMetricsF>
#include <QPaintEvent>
#include <QPainter>
#include <QtMath>
#include <algorithm>
#include <cmath>

namespace autoviz::widget {

Watermark::Watermark(QWidget* parent) : QWidget(parent) {
  setObjectName(QStringLiteral("AutovizWatermark"));
  setProperty("autovizRole", QStringLiteral("watermark"));
  setAttribute(Qt::WA_TransparentForMouseEvents);
  setAttribute(Qt::WA_NoSystemBackground);
  setFocusPolicy(Qt::NoFocus);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setAccessibleName(tr("Watermark"));
  UpdateAccessibility();
}

void Watermark::SetText(const QString& text) {
  if (text_ == text) {
    return;
  }
  text_ = text;
  InvalidateTile();
  UpdateAccessibility();
  update();
}

void Watermark::SetOpacity(qreal opacity) {
  if (!std::isfinite(opacity)) {
    return;
  }
  const qreal bounded = std::clamp(opacity, 0.0, 1.0);
  if (qFuzzyCompare(opacity_ + 1.0, bounded + 1.0)) {
    return;
  }
  opacity_ = bounded;
  InvalidateTile();
  update();
}

void Watermark::SetAngleDegrees(qreal angle_degrees) {
  if (!std::isfinite(angle_degrees)) {
    return;
  }
  const qreal bounded = std::clamp(angle_degrees, -180.0, 180.0);
  if (qFuzzyCompare(angle_degrees_ + 181.0, bounded + 181.0)) {
    return;
  }
  angle_degrees_ = bounded;
  InvalidateTile();
  update();
}

void Watermark::SetTileSpacing(const QSize& spacing) {
  const QSize bounded(std::clamp(spacing.width(), 0, 512),
                      std::clamp(spacing.height(), 0, 512));
  if (tile_spacing_ == bounded) {
    return;
  }
  tile_spacing_ = bounded;
  InvalidateTile();
  update();
}

void Watermark::SetTone(style::Tone tone) {
  if (tone_ == tone) {
    return;
  }
  tone_ = tone;
  InvalidateTile();
  update();
}

QSize Watermark::sizeHint() const { return QSize(320, 180); }

void Watermark::changeEvent(QEvent* event) {
  QWidget::changeEvent(event);
  switch (event->type()) {
    case QEvent::ApplicationFontChange:
    case QEvent::ApplicationPaletteChange:
    case QEvent::FontChange:
    case QEvent::PaletteChange:
    case QEvent::StyleChange:
      InvalidateTile();
      break;
    default:
      break;
  }
}

void Watermark::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  if (text_.isEmpty() || opacity_ <= 0.0) {
    return;
  }
  const qreal device_pixel_ratio = devicePixelRatioF();
  if (tile_.isNull() ||
      !qFuzzyCompare(tile_device_pixel_ratio_, device_pixel_ratio)) {
    RebuildTile(device_pixel_ratio);
  }
  if (tile_.isNull()) {
    return;
  }
  QPainter painter(this);
  painter.drawTiledPixmap(rect(), tile_);
}

void Watermark::InvalidateTile() {
  tile_ = QPixmap();
  tile_device_pixel_ratio_ = 0.0;
}

void Watermark::RebuildTile(qreal device_pixel_ratio) {
  const QFontMetricsF metrics(font());
  const QRectF text_bounds = metrics.boundingRect(text_);
  const qreal radians = qDegreesToRadians(angle_degrees_);
  const qreal cosine = std::abs(std::cos(radians));
  const qreal sine = std::abs(std::sin(radians));
  const qreal rotated_width =
      cosine * text_bounds.width() + sine * text_bounds.height();
  const qreal rotated_height =
      sine * text_bounds.width() + cosine * text_bounds.height();
  const QSize logical_size(
      qMax(1, qCeil(rotated_width) + tile_spacing_.width()),
      qMax(1, qCeil(rotated_height) + tile_spacing_.height()));
  const QSize pixel_size(
      qMax(1, qCeil(logical_size.width() * device_pixel_ratio)),
      qMax(1, qCeil(logical_size.height() * device_pixel_ratio)));

  tile_ = QPixmap(pixel_size);
  tile_.setDevicePixelRatio(device_pixel_ratio);
  tile_.fill(Qt::transparent);
  QPainter painter(&tile_);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setRenderHint(QPainter::TextAntialiasing);
  QColor color = GetThemeTokens(this, tone_).text_muted;
  color.setAlphaF(color.alphaF() * opacity_);
  painter.setPen(color);
  painter.setFont(font());
  painter.translate(logical_size.width() / 2.0, logical_size.height() / 2.0);
  painter.rotate(angle_degrees_);
  painter.drawText(
      QRectF(-text_bounds.width() / 2.0, -text_bounds.height() / 2.0,
             text_bounds.width(), text_bounds.height()),
      Qt::AlignCenter, text_);
  tile_device_pixel_ratio_ = device_pixel_ratio;
}

void Watermark::UpdateAccessibility() {
  setAccessibleDescription(text_.isEmpty() ? tr("No watermark text")
                                           : tr("Watermark: %1").arg(text_));
}

}  // namespace autoviz::widget
