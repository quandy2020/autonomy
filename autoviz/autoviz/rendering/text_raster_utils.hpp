/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file text_raster_utils.hpp
 * @brief Rasterize label text to an RGBA @c QImage for billboard overlays.
 *
 * Used by TEXT markers and screen-facing labels drawn via @ref SceneOverlay
 * textured quads or Ogre movable text paths.
 *
 * @see SceneOverlay::addViewFacingTexturedQuad()
 * @see RasterizeTextLabel()
 */

#pragma once

#include <QColor>
#include <QImage>
#include <QString>

namespace autoviz {
namespace rendering {

/**
 * @brief Rasterizes label text to an RGBA image with a transparent background.
 *
 * @param text UTF-16 / Qt string to draw.
 * @param color Foreground text color (alpha respected).
 * @param pixel_height Approximate glyph height in pixels (default 32).
 * @return Image sized to fit the text; empty if @p text is empty.
 *
 * @note Caller owns the returned image; typically uploaded as a texture for
 *       a view-facing quad.
 */
QImage RasterizeTextLabel(const QString& text, const QColor& color,
                          int pixel_height = 32);

}  // namespace rendering
}  // namespace autoviz
