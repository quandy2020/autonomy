/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QImage>
#include <QString>

namespace autoviz {
namespace rendering {

/** Rasterize label text to RGBA image (transparent background). */
QImage RasterizeTextLabel(const QString& text, const QColor& color,
                          int pixel_height = 32);

}  // namespace rendering
}  // namespace autoviz
