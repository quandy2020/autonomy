/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>

#include "autoviz/rendering/render_settings.hpp"

#ifdef AUTOVIZ_USE_OGRE
#include "autoviz/rendering/objects/ogre_point_cloud.hpp"
#endif

namespace autoviz {
namespace rendering {

/** Parse rviz-style style name (Points/Squares/Flat Squares/Spheres/Tiles/Boxes). */
PointCloudStyle parsePointCloudStyle(const std::string& value);

#ifdef AUTOVIZ_USE_OGRE
OgrePointCloud::RenderMode toOgrePointCloudRenderMode(PointCloudStyle style);
#endif

}  // namespace rendering
}  // namespace autoviz
