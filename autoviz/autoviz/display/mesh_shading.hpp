/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file mesh_shading.hpp
 * @brief Per-face shade helper for unlit solid meshes so shaft/cone parts of
 *        an arrow remain visually distinct.
 *
 * Matches the lit look of @c rviz_rendering::Shape (ambient + diffuse) without
 * requiring a lit material in the overlay path. Shading is two-sided: the
 * overlay draws back faces, so lighting uses how much the face plane faces
 * the key light rather than winding order.
 *
 * @see shadeTriangleColor
 * @see appendSolidArrowMeshes
 */

#pragma once

#include <QColor>
#include <QVector3D>

namespace autoviz {
namespace display {

/**
 * @brief Returns @p color modulated by a simple key-light shade for triangle
 *        @p a–@p b–@p c.
 *
 * Without shading, a shaft and cone drawn in one flat color merge into a
 * single silhouette. The result keeps hue/saturation while darkening faces
 * that face away from the key light.
 *
 * @param color Base RGBA for the mesh.
 * @param a First triangle vertex (world / overlay frame).
 * @param b Second triangle vertex.
 * @param c Third triangle vertex.
 * @return Shaded @c QColor suitable for @c ColoredMeshInstance.
 *
 * @note Intended for unlit materials only; lit pipelines should use real
 *       normals / lights instead.
 *
 * @see appendSolidArrowMeshes
 */
QColor shadeTriangleColor(const QColor& color, const QVector3D& a,
                          const QVector3D& b, const QVector3D& c);

}  // namespace display
}  // namespace autoviz
