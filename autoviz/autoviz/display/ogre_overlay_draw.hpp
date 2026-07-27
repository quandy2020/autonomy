/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <array>
#include <string>
#include <vector>

#include <QColor>
#include <QQuaternion>
#include <QVector3D>

namespace autoviz {
namespace common {
class DisplayContext;
}  // namespace common
namespace rendering {
class SceneOverlay;
}  // namespace rendering

namespace display {

struct LineSegment3D {
  QVector3D a;
  QVector3D b;
  QColor color;
};

/** Ogre persistent ManualObject lines when ogre_scene_host is set. */
bool drawLineSegmentsOgreOrGl(common::DisplayContext* context,
                              rendering::SceneOverlay& scene,
                              const std::string& display_name,
                              const std::vector<LineSegment3D>& segments);

/** rviz BillboardLine polyline when line_width > 0 and ogre_scene_host is set. */
bool drawBillboardStripOgreOrGl(common::DisplayContext* context,
                                rendering::SceneOverlay& scene,
                                const std::string& display_name,
                                const std::vector<QVector3D>& points,
                                const QColor& color, float line_width);

/** Shaft + cone head (rviz Arrow equivalent). Diameter 0 = auto from length. */
bool drawArrowOgreOrGl(common::DisplayContext* context,
                       rendering::SceneOverlay& scene,
                       const std::string& display_name, const QVector3D& start,
                       const QVector3D& end, const QColor& color,
                       float head_fraction = 0.2f, float shaft_diameter = 0.f,
                       float head_diameter = 0.f);

/** Vector arrow with min length + scale; origin dot when length ~ 0. */
bool drawVectorArrowOgreOrGl(common::DisplayContext* context,
                             rendering::SceneOverlay& scene,
                             const std::string& display_name,
                             const QVector3D& origin, const QVector3D& vector,
                             const QColor& color, float min_length,
                             float scale, float shaft_diameter = 0.f,
                             float head_diameter = 0.f);

/** rviz WrenchVisual when ogre_scene_host is set. */
bool drawWrenchOgreOrGl(common::DisplayContext* context,
                        rendering::SceneOverlay& scene,
                        const std::string& display_name, const QVector3D& origin,
                        const QVector3D& force, const QVector3D& torque,
                        const QColor& force_color, const QColor& torque_color,
                        float force_scale, float torque_scale, float width);

/** rviz ScrewVisual when ogre_scene_host is set. */
bool drawScrewOgreOrGl(common::DisplayContext* context,
                       rendering::SceneOverlay& scene,
                       const std::string& display_name, const QVector3D& origin,
                       const QVector3D& linear, const QVector3D& angular,
                       const QColor& linear_color, const QColor& angular_color,
                       float linear_scale, float angular_scale, float width,
                       bool hide_small_values);

/** rviz CovarianceVisual when ogre_scene_host is set. */
bool drawCovarianceOgreOrGl(
    common::DisplayContext* context, rendering::SceneOverlay& scene,
    const std::string& display_name, const QVector3D& position,
    const QQuaternion& pose_orientation, const QQuaternion& frame_orientation,
    const std::array<double, 36>& covariance, const QColor& position_color,
    float position_scale, float orientation_scale, float orientation_offset,
    bool visible);

}  // namespace display
}  // namespace autoviz
