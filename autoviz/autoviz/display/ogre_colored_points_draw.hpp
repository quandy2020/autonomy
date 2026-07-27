/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

struct ColoredPoint3D {
  QVector3D position;
  QColor color;
};

/** Ogre OgrePointCloud path when ogre_scene_host is set; otherwise SceneOverlay GL. */
bool drawColoredPointsOgreOrGl(common::DisplayContext* context,
                               rendering::SceneOverlay& scene,
                               const std::string& display_name,
                               const std::string& display_type, float point_size,
                               rendering::PointCloudStyle style,
                               const std::vector<ColoredPoint3D>& points,
                               bool per_point_pick = true);

}  // namespace display
}  // namespace autoviz
