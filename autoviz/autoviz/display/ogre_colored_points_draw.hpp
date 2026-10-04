/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_colored_points_draw.hpp
 * @brief Draw helper for per-point colored 3D point clouds (Ogre or GL).
 *
 * When @c DisplayContext::ogre_scene_host is set, points go through the Ogre
 * @c OgrePointCloud path; otherwise they fall back to
 * @ref rendering::SceneOverlay immediate GL.
 *
 * Used by @ref PointCloud2Display, @ref LaserScanDisplay, depth cloud, and
 * other scalar/point displays that share the colored-points pipeline.
 *
 * @see ColoredPoint3D
 * @see rendering::PointCloudStyle
 * @see PointCloud2Display
 */

#pragma once

#include <string>
#include <vector>

#include <QColor>
#include <QVector3D>

#include "autoviz/rendering/render_settings.hpp"

namespace autoviz {
namespace common {
class DisplayContext;
class SelectionHandler;
}  // namespace common
namespace rendering {
class SceneOverlay;
}  // namespace rendering

namespace display {

/**
 * @struct ColoredPoint3D
 * @brief One point with world position and RGBA color.
 */
struct ColoredPoint3D {
  QVector3D position; /**< World / fixed-frame position. */
  QColor color;       /**< Per-point color (alpha respected by style). */
};

/**
 * @brief Draws colored points via Ogre when available, else SceneOverlay GL.
 *
 * @param context Display context (view, Ogre host, selection); may be
 *        @c nullptr (returns @c false).
 * @param scene GL overlay fallback when Ogre is unavailable.
 * @param display_name Stable id for Ogre object naming / picking.
 * @param display_type Catalog type string (e.g. @c "PointCloud2") for pick
 *        metadata.
 * @param point_size Pixel or world size depending on @p style.
 * @param style Point cloud render style (@c Points, @c FlatSquares, …).
 * @param points Colored points to draw (may be empty → no-op success).
 * @param per_point_pick When @c true, registers pick handles for selection.
 * @return @c true if the chosen backend accepted the draw.
 *
 * @note Prefer calling from @c Display::onDraw on the render thread.
 * @see rendering::PointCloudStyle
 */
bool drawColoredPointsOgreOrGl(common::DisplayContext* context,
                               rendering::SceneOverlay& scene,
                               const std::string& display_name,
                               const std::string& display_type, float point_size,
                               rendering::PointCloudStyle style,
                               const std::vector<ColoredPoint3D>& points,
                               bool per_point_pick = true);

}  // namespace display
}  // namespace autoviz
