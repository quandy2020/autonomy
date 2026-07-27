/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <vector>

#include <QColor>
#include <QVector3D>

#include "autoviz/display/ogre_mesh_draw.hpp"

namespace autoviz {
namespace display {

/** Append rviz-style solid arrow (cylinder shaft + cone head) along start→end. */
void appendSolidArrowMeshes(std::vector<ColoredMeshInstance>* meshes,
                            const QVector3D& start, const QVector3D& end,
                            const QColor& color, float head_fraction = 0.2f,
                            float shaft_diameter = 0.f, float head_diameter = 0.f);

}  // namespace display
}  // namespace autoviz
