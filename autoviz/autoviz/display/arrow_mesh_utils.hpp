/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file arrow_mesh_utils.hpp
 * @brief Helpers that append RViz-style solid arrow meshes (cylinder shaft +
 *        cone head) to an overlay mesh list.
 *
 * Used by effort / IMU / accel and other displays that need a directional
 * 3D arrow without owning a dedicated Shape object.
 *
 * @see appendSolidArrowMeshes
 * @see ColoredMeshInstance
 * @see mesh_shading.hpp
 */

#pragma once

#include <vector>

#include <QColor>
#include <QVector3D>

#include "autoviz/display/ogre_mesh_draw.hpp"

namespace autoviz {
namespace display {

/**
 * @brief Appends a solid arrow (cylinder shaft + cone head) along
 *        @p start → @p end into @p meshes.
 *
 * Matches RViz arrow proportions: shaft then head along the segment. When
 * diameters are @c 0, defaults derived from segment length are used.
 *
 * @param[out] meshes Destination list of colored mesh instances.
 * @param start Shaft base position in the same frame as @p end.
 * @param end Arrow tip position.
 * @param color RGBA applied to shaft and head (may be further shaded by
 *        @ref shadeTriangleColor at draw time).
 * @param head_fraction Fraction of total length reserved for the cone head
 *        (default @c 0.2).
 * @param shaft_diameter Shaft diameter; @c 0 selects an automatic size.
 * @param head_diameter Cone base diameter; @c 0 selects an automatic size.
 *
 * @note Degenerate @p start≈@p end segments should be skipped by callers.
 *
 * @see shadeTriangleColor
 * @see ColoredMeshInstance
 */
void appendSolidArrowMeshes(std::vector<ColoredMeshInstance>* meshes,
                            const QVector3D& start, const QVector3D& end,
                            const QColor& color, float head_fraction = 0.2f,
                            float shaft_diameter = 0.f, float head_diameter = 0.f);

}  // namespace display
}  // namespace autoviz
