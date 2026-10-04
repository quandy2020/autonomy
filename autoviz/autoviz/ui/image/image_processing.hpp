/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file image_processing.hpp
 * @brief Stateless image display transforms: rotate/flip, colormap, overlay blend.
 *
 * Applied by @ref ImagePanel when composing the frame shown in
 * @ref ImageViewWidget.
 *
 * @see ImagePanelConfig
 * @see ImageOverlayConfig
 */

#pragma once

#include <QImage>

#include "autoviz/ui/image/image_types.hpp"

namespace autoviz {
namespace image {

/**
 * @brief Applies horizontal/vertical flip and discrete rotation.
 *
 * @param source Input image.
 * @param flip_horizontal Mirror left↔right when @c true.
 * @param flip_vertical Mirror top↔bottom when @c true.
 * @param rotation Clockwise rotation from @ref ImageRotation.
 * @return Transformed image (may share data when no-op).
 */
QImage applyDisplayTransform(const QImage& source,
                           bool flip_horizontal, bool flip_vertical,
                           ImageRotation rotation);

/**
 * @brief Remaps intensity into a false-color or grayscale display mode.
 *
 * @param source Input image (typically single-channel or RGB).
 * @param mode Colormap / grayscale mode; @ref ImageColorMode::kOff returns
 *        @p source unchanged.
 * @param min_value Intensity mapped to the low end of the colormap.
 * @param max_value Intensity mapped to the high end of the colormap.
 * @return Color-mapped image.
 */
QImage applyColorMode(const QImage& source, ImageColorMode mode,
                      double min_value, double max_value);

/**
 * @brief Composites an overlay image onto @p base using overlay style.
 *
 * Honors opacity, @ref ImageBlendMode, and @ref ImagePixelAlpha from @p config.
 *
 * @param base Bottom layer (main camera frame).
 * @param overlay Top layer (must be resized by caller if needed).
 * @param config Overlay blend / opacity / pixel-alpha options.
 * @return Composited result (size of @p base).
 */
QImage compositeOverlay(const QImage& base, const QImage& overlay,
                        const ImageOverlayConfig& config);

}  // namespace image
}  // namespace autoviz
