/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QImage>

#include "autoviz/ui/image/image_types.hpp"

namespace autoviz {
namespace image {

QImage applyDisplayTransform(const QImage& source,
                           bool flip_horizontal, bool flip_vertical,
                           ImageRotation rotation);

QImage applyColorMode(const QImage& source, ImageColorMode mode,
                      double min_value, double max_value);

QImage compositeOverlay(const QImage& base, const QImage& overlay,
                        const ImageOverlayConfig& config);

}  // namespace image
}  // namespace autoviz
