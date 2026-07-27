/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/point_cloud_style_utils.hpp"

#include <algorithm>
#include <cctype>

namespace autoviz {
namespace rendering {
namespace {

std::string normalizeStyleKey(std::string value) {
  value.erase(std::remove_if(value.begin(), value.end(), ::isspace), value.end());
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

}  // namespace

PointCloudStyle parsePointCloudStyle(const std::string& value) {
  const std::string key = normalizeStyleKey(value);
  if (key == "points" || key == "point") {
    return PointCloudStyle::kPoints;
  }
  if (key == "flatsquares" || key == "flat_squares" || key == "flatsquare") {
    return PointCloudStyle::kFlatSquares;
  }
  if (key == "spheres" || key == "sphere") {
    return PointCloudStyle::kSpheres;
  }
  if (key == "tiles" || key == "tile") {
    return PointCloudStyle::kTiles;
  }
  if (key == "boxes" || key == "box") {
    return PointCloudStyle::kBoxes;
  }
  return PointCloudStyle::kSquares;
}

#ifdef AUTOVIZ_USE_OGRE

OgrePointCloud::RenderMode toOgrePointCloudRenderMode(PointCloudStyle style) {
  switch (style) {
    case PointCloudStyle::kPoints:
      return OgrePointCloud::kPoints;
    case PointCloudStyle::kFlatSquares:
      return OgrePointCloud::kFlatSquares;
    case PointCloudStyle::kSpheres:
      return OgrePointCloud::kSpheres;
    case PointCloudStyle::kTiles:
      return OgrePointCloud::kTiles;
    case PointCloudStyle::kBoxes:
      return OgrePointCloud::kBoxes;
    case PointCloudStyle::kSquares:
    default:
      return OgrePointCloud::kSquares;
  }
}

#endif

}  // namespace rendering
}  // namespace autoviz
