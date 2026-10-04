/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file render_settings.hpp
 * @brief Shared render enums and settings used by OpenGL and Ogre backends.
 *
 * Holds the optional built-in reference grid configuration and the point-cloud
 * draw-style enum shared by @ref GridRenderer, @ref OgreGrid, and
 * @ref OgrePointCloud / style parsers.
 *
 * @see GridRenderer
 * @see OgreGrid
 * @see parsePointCloudStyle()
 */

#pragma once

#include <cstdint>

#include <QColor>

namespace autoviz {
namespace rendering {

/**
 * @struct ReferenceGridSettings
 * @brief Optional built-in OpenGL/Ogre ground grid (REP-103 Z-up XY plane).
 *
 * @note Autoviz keeps this grid off by default; RViz2-style users typically
 *       enable a separate Grid Display instead of this built-in overlay.
 *
 * @see GridRenderer::setReferenceGridSettings()
 * @see OgreGrid::setSettings()
 */
struct ReferenceGridSettings {
  /**
   * @brief Total cells along each axis on the ground plane.
   *
   * Matches rviz «Plane Cell Count» (default 10 → 10×10 cells).
   */
  int plane_cell_count = 10;

  /** @brief World-space length of one cell edge (meters). */
  float cell_length = 1.f;

  /** @brief Line color of the grid (RGB; alpha taken from @ref alpha). */
  QColor color{80, 80, 80};

  /** @brief Grid line opacity in \[0, 1\]. */
  float alpha = 1.f;

  /** @brief When @c true, draw RGB axes at the origin. */
  bool show_axes = true;

  /**
   * @brief Length of the RGB origin axes (rviz-style, roughly one cell).
   * @see show_axes
   */
  float axis_length = 1.f;

  /**
   * @brief Equality comparison of all fields (including @c QColor).
   * @param other Settings to compare against.
   * @return @c true if every field matches.
   */
  bool operator==(const ReferenceGridSettings& other) const {
    return plane_cell_count == other.plane_cell_count &&
           cell_length == other.cell_length && color == other.color &&
           alpha == other.alpha && show_axes == other.show_axes &&
           axis_length == other.axis_length;
  }

  /**
   * @brief Inequality comparison.
   * @param other Settings to compare against.
   * @return @c true if any field differs.
   */
  bool operator!=(const ReferenceGridSettings& other) const {
    return !(*this == other);
  }
};

/**
 * @enum PointCloudStyle
 * @brief Point cloud draw style (rviz @c PointCloud::RenderMode).
 *
 * Parsed from display property strings via @ref parsePointCloudStyle() and
 * mapped to @ref OgrePointCloud::RenderMode when Ogre is enabled.
 *
 * @see parsePointCloudStyle()
 * @see toOgrePointCloudRenderMode()
 */
enum class PointCloudStyle {
  kPoints = 0,   /**< GL / Ogre point primitives. */
  kSquares,      /**< Camera-facing squares (billboard). */
  kFlatSquares,  /**< Flat squares on a fixed plane. */
  kSpheres,      /**< Sphere impostors / geometry. */
  kTiles,        /**< Tile billboards. */
  kBoxes,        /**< Box geometry per point. */
};

}  // namespace rendering
}  // namespace autoviz
