/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QPixmap>
#include <QSize>
#include <QString>
#include <QWidget>

#include "autoviz/common/widget/foundation/theme_tokens.hpp"

class QEvent;
class QPaintEvent;

namespace autoviz::widget {

// Lightweight, mouse-transparent repeated text overlay. A device-pixel-ratio
// aware tile cache keeps large overlays inexpensive to repaint.
class Watermark final : public QWidget {
 public:
  explicit Watermark(QWidget* parent = nullptr);

  QString text() const { return text_; }
  void SetText(const QString& text);

  qreal opacity() const noexcept { return opacity_; }
  void SetOpacity(qreal opacity);

  qreal angle_degrees() const noexcept { return angle_degrees_; }
  void SetAngleDegrees(qreal angle_degrees);

  QSize tile_spacing() const noexcept { return tile_spacing_; }
  void SetTileSpacing(const QSize& spacing);

  style::Tone tone() const noexcept { return tone_; }
  void SetTone(style::Tone tone);

  QSize sizeHint() const override;

 protected:
  void changeEvent(QEvent* event) override;
  void paintEvent(QPaintEvent* event) override;

 private:
  void InvalidateTile();
  void RebuildTile(qreal device_pixel_ratio);
  void UpdateAccessibility();

  QString text_;
  QSize tile_spacing_{64, 48};
  QPixmap tile_;
  style::Tone tone_ = style::Tone::Shell;
  qreal opacity_ = 0.16;
  qreal angle_degrees_ = -24.0;
  qreal tile_device_pixel_ratio_ = 0.0;
};

}  // namespace autoviz::widget
