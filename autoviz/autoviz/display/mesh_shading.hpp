/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QVector3D>

namespace autoviz {
namespace display {

/** Per-face shade for a solid mesh drawn with an unlit material, matching the
 *  lit look of rviz_rendering::Shape (ambient plus a diffuse term). Without it a
 *  shaft and a cone in one colour merge into a single flat silhouette.
 *
 *  Shading is two-sided: the overlay draws back faces as well, so a face is lit
 *  by how much its plane faces the key light rather than by its winding.
 */
QColor shadeTriangleColor(const QColor& color, const QVector3D& a,
                          const QVector3D& b, const QVector3D& c);

}  // namespace display
}  // namespace autoviz
