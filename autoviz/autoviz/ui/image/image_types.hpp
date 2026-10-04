/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file image_types.hpp
 * @brief Shared enums and config structs for the Image panel.
 *
 * Consumed by @ref ImagePanel, @ref ImageSettingsWidget, image processing,
 * and session persist IO (@ref image_config_io.hpp).
 *
 * @see ImagePanelConfig
 * @see ImageOverlayConfig
 */

#pragma once

#include <QColor>
#include <QString>
#include <QStringList>
#include <QVector>

namespace autoviz {
namespace image {

/**
 * @enum ImageRotation
 * @brief Discrete clockwise rotation applied before display.
 */
enum class ImageRotation {
  k0 = 0,     /**< No rotation. */
  k90 = 90,   /**< 90° clockwise. */
  k180 = 180, /**< 180°. */
  k270 = 270, /**< 270° clockwise. */
};

/**
 * @enum ImageColorMode
 * @brief Intensity remapping / false-color mode for the main image.
 */
enum class ImageColorMode {
  kOff = 0,   /**< Pass-through RGB / original. */
  kTurbo,     /**< Turbo colormap. */
  kRainbow,   /**< Rainbow colormap. */
  kGrayscale, /**< Single-channel grayscale display. */
};

/**
 * @enum ImageBlendMode
 * @brief How an overlay image is composited onto the base frame.
 */
enum class ImageBlendMode {
  kAlpha = 0, /**< Standard alpha blending with opacity. */
  kAdd,       /**< Additive blend. */
};

/**
 * @enum ImagePixelAlpha
 * @brief Per-pixel alpha heuristics for overlay sources.
 */
enum class ImagePixelAlpha {
  kNone = 0,          /**< Use source alpha as-is. */
  kWhiteTransparent,  /**< Treat near-white pixels as transparent. */
};

/**
 * @struct ImageOverlayConfig
 * @brief One overlay layer: channel, opacity, blend, and enable flag.
 */
struct ImageOverlayConfig {
  QString channel;  /**< Overlay image channel name. */
  double opacity = 0.5;  /**< Blend opacity in \[0, 1\]. */
  ImageBlendMode blend_mode = ImageBlendMode::kAlpha;  /**< Composite mode. */
  ImagePixelAlpha pixel_alpha = ImagePixelAlpha::kNone;  /**< Pixel-alpha heuristic. */
  bool enabled = true;  /**< When @c false, layer is skipped. */
};

/**
 * @struct ImagePanelConfig
 * @brief Full persisted configuration for an @ref ImagePanel instance.
 *
 * Includes source channels, display transforms, overlays, annotation / marker
 * lists, and UI chrome state (@c settings_visible).
 */
struct ImagePanelConfig {
  QString title = QStringLiteral("Image");  /**< Panel / dock title. */
  QString image_channel;                    /**< Main image channel. */
  QString calibration_channel;              /**< Optional CameraInfo channel. */
  bool strict_time_sync = false;            /**< Sync overlays to main stamp. */
  bool flip_horizontal = false;             /**< Mirror left↔right. */
  bool flip_vertical = false;               /**< Mirror top↔bottom. */
  ImageRotation rotation = ImageRotation::k0;  /**< Discrete rotation. */
  ImageColorMode color_mode = ImageColorMode::kOff;  /**< False-color mode. */
  double color_min = 0.0;                   /**< Colormap low end. */
  double color_max = 255.0;                 /**< Colormap high end. */
  QVector<ImageOverlayConfig> overlays;     /**< Overlay layers. */
  QStringList annotation_channels;          /**< Annotation source channels. */
  QColor background_color = QColor(Qt::black);  /**< View letterbox color. */
  double label_scale = 1.0;                 /**< Annotation text scale. */
  QString click_publish_channel;            /**< Pixel-click output channel. */
  QString hover_publish_channel;            /**< Pixel-hover output channel. */
  bool enable_undistort = false;            /**< Undistort using CameraInfo. */
  QStringList marker_channels;              /**< Marker projection channels. */
  QStringList point_cloud_channels;         /**< PointCloud2 projection channels. */
  bool settings_visible = false;            /**< Settings pane visibility. */
};

}  // namespace image
}  // namespace autoviz
