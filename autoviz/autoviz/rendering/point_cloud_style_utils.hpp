/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file point_cloud_style_utils.hpp
 * @brief Parse rviz-style point-cloud style names and map to Ogre render modes.
 *
 * Bridges display-property strings (@c "Points", @c "Flat Squares", …) to
 * @ref PointCloudStyle and, when Ogre is enabled, to
 * @ref OgrePointCloud::RenderMode.
 *
 * @see PointCloudStyle
 * @see OgrePointCloud
 */

#pragma once

#include <string>

#include "autoviz/rendering/render_settings.hpp"

#include "autoviz/rendering/objects/ogre_point_cloud.hpp"

namespace autoviz {
namespace rendering {

/**
 * @brief Parses an rviz-style style name into @ref PointCloudStyle.
 *
 * Accepted names (case-sensitive, matching RViz2 property strings):
 * @c Points, @c Squares, @c Flat Squares, @c Spheres, @c Tiles, @c Boxes.
 * Unknown values fall back to @ref PointCloudStyle::kPoints.
 *
 * @param value Style name from a display property or session config.
 * @return Corresponding @ref PointCloudStyle enum value.
 *
 * @see toOgrePointCloudRenderMode()
 */
PointCloudStyle parsePointCloudStyle(const std::string& value);

/**
 * @brief Maps @ref PointCloudStyle to @ref OgrePointCloud::RenderMode.
 *
 * @param style Autoviz style enum.
 * @return Matching Ogre point-cloud render mode.
 *
 * @see parsePointCloudStyle()
 * @see OgrePointCloud::setRenderMode()
 */
OgrePointCloud::RenderMode toOgrePointCloudRenderMode(PointCloudStyle style);

}  // namespace rendering
}  // namespace autoviz
