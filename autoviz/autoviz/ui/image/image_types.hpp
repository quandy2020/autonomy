/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QString>
#include <QStringList>
#include <QVector>

namespace autoviz {
namespace image {

enum class ImageRotation {
  k0 = 0,
  k90 = 90,
  k180 = 180,
  k270 = 270,
};

enum class ImageColorMode {
  kOff = 0,
  kTurbo,
  kRainbow,
  kGrayscale,
};

enum class ImageBlendMode {
  kAlpha = 0,
  kAdd,
};

enum class ImagePixelAlpha {
  kNone = 0,
  kWhiteTransparent,
};

struct ImageOverlayConfig {
  QString channel;
  double opacity = 0.5;
  ImageBlendMode blend_mode = ImageBlendMode::kAlpha;
  ImagePixelAlpha pixel_alpha = ImagePixelAlpha::kNone;
  bool enabled = true;
};

struct ImagePanelConfig {
  QString title = QStringLiteral("Image");
  QString image_channel = QStringLiteral("/fake/image");
  QString calibration_channel;
  bool strict_time_sync = false;
  bool flip_horizontal = false;
  bool flip_vertical = false;
  ImageRotation rotation = ImageRotation::k0;
  ImageColorMode color_mode = ImageColorMode::kOff;
  double color_min = 0.0;
  double color_max = 255.0;
  QVector<ImageOverlayConfig> overlays;
  QStringList annotation_channels;
  QColor background_color = QColor(Qt::black);
  double label_scale = 1.0;
  QString click_publish_channel;
  QString hover_publish_channel;
  bool enable_undistort = false;
  QStringList marker_channels;
  bool settings_visible = false;
};

}  // namespace image
}  // namespace autoviz
