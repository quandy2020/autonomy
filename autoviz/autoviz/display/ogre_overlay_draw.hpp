/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_overlay_draw.hpp
 * @brief Ogre/GL draw helpers for lines, arrows, wrenches, screws, covariance.
 *
 * Thin wrappers that prefer RViz-style Ogre visuals
 * (@c BillboardLine, @c Arrow, @c WrenchVisual, @c ScrewVisual,
 * @c CovarianceVisual) when @c ogre_scene_host is set, and fall back to
 * @ref rendering::SceneOverlay primitives otherwise.
 *
 * Shared by pose, path, twist, wrench, IMU, effort, and covariance displays.
 *
 * @see drawArrowOgreOrGl()
 * @see drawWrenchOgreOrGl()
 * @see drawCovarianceOgreOrGl()
 */

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

/**
 * @struct LineSegment3D
 * @brief Colored line segment between two world-space endpoints.
 */
struct LineSegment3D {
  QVector3D a; /**< Segment start. */
  QVector3D b; /**< Segment end. */
  QColor color; /**< Segment color. */
};

/**
 * @brief Draws thin line segments via Ogre ManualObject or GL.
 *
 * @param context Display context (Ogre host optional).
 * @param scene GL overlay fallback.
 * @param display_name Stable object-name prefix.
 * @param segments Colored segments to draw.
 * @return @c true if a backend accepted the draw.
 */
bool drawLineSegmentsOgreOrGl(common::DisplayContext* context,
                              rendering::SceneOverlay& scene,
                              const std::string& display_name,
                              const std::vector<LineSegment3D>& segments);

/**
 * @brief Draws a width-aware polyline (RViz BillboardLine) when @p line_width > 0.
 *
 * @param context Display context.
 * @param scene GL overlay fallback.
 * @param display_name Stable object-name prefix.
 * @param points Ordered polyline vertices.
 * @param color Strip color.
 * @param line_width World-space width; @c 0 falls back to thin lines.
 * @return @c true if a backend accepted the draw.
 *
 * @note Used by @ref PathDisplay when Line Style is Billboards.
 */
bool drawBillboardStripOgreOrGl(common::DisplayContext* context,
                                rendering::SceneOverlay& scene,
                                const std::string& display_name,
                                const std::vector<QVector3D>& points,
                                const QColor& color, float line_width);

/**
 * @brief Draws a shaft + cone arrow (RViz Arrow equivalent).
 *
 * @param context Display context.
 * @param scene GL overlay fallback.
 * @param display_name Stable object-name prefix.
 * @param start Arrow base (world).
 * @param end Arrow tip (world).
 * @param color Arrow color.
 * @param head_fraction Cone length as a fraction of total length.
 * @param shaft_diameter Shaft diameter; @c 0 = auto from length.
 * @param head_diameter Head diameter; @c 0 = auto from length.
 * @return @c true if a backend accepted the draw.
 */
bool drawArrowOgreOrGl(common::DisplayContext* context,
                       rendering::SceneOverlay& scene,
                       const std::string& display_name, const QVector3D& start,
                       const QVector3D& end, const QColor& color,
                       float head_fraction = 0.2f, float shaft_diameter = 0.f,
                       float head_diameter = 0.f);

/**
 * @brief Draws a vector as an arrow with minimum length and uniform scale.
 *
 * When @p vector length is near zero, may draw an origin marker instead of a
 * degenerate arrow.
 *
 * @param context Display context.
 * @param scene GL overlay fallback.
 * @param display_name Stable object-name prefix.
 * @param origin Arrow origin (world).
 * @param vector Direction and magnitude before @p scale.
 * @param color Arrow color.
 * @param min_length Minimum drawn length after scaling.
 * @param scale Multiplier applied to @p vector.
 * @param shaft_diameter Shaft diameter; @c 0 = auto.
 * @param head_diameter Head diameter; @c 0 = auto.
 * @return @c true if a backend accepted the draw.
 */
bool drawVectorArrowOgreOrGl(common::DisplayContext* context,
                             rendering::SceneOverlay& scene,
                             const std::string& display_name,
                             const QVector3D& origin, const QVector3D& vector,
                             const QColor& color, float min_length,
                             float scale, float shaft_diameter = 0.f,
                             float head_diameter = 0.f);

/**
 * @brief Draws force + torque as an RViz WrenchVisual when Ogre is available.
 *
 * @param context Display context.
 * @param scene GL overlay fallback.
 * @param display_name Stable object-name prefix.
 * @param origin Wrench origin (world).
 * @param force Force vector.
 * @param torque Torque vector.
 * @param force_color Color for the force arrow.
 * @param torque_color Color for the torque arrow / arc.
 * @param force_scale Scale applied to force.
 * @param torque_scale Scale applied to torque.
 * @param width Arrow / arc width.
 * @return @c true if a backend accepted the draw.
 *
 * @see WrenchDisplay
 */
bool drawWrenchOgreOrGl(common::DisplayContext* context,
                        rendering::SceneOverlay& scene,
                        const std::string& display_name, const QVector3D& origin,
                        const QVector3D& force, const QVector3D& torque,
                        const QColor& force_color, const QColor& torque_color,
                        float force_scale, float torque_scale, float width);

/**
 * @brief Draws linear + angular as an RViz ScrewVisual when Ogre is available.
 *
 * @param context Display context.
 * @param scene GL overlay fallback.
 * @param display_name Stable object-name prefix.
 * @param origin Screw origin (world).
 * @param linear Linear velocity / screw linear part.
 * @param angular Angular velocity / screw angular part.
 * @param linear_color Color for the linear arrow.
 * @param angular_color Color for the angular arrow / arc.
 * @param linear_scale Scale for linear.
 * @param angular_scale Scale for angular.
 * @param width Arrow / arc width.
 * @param hide_small_values When @c true, suppresses near-zero components.
 * @return @c true if a backend accepted the draw.
 *
 * @see TwistStampedDisplay
 */
bool drawScrewOgreOrGl(common::DisplayContext* context,
                       rendering::SceneOverlay& scene,
                       const std::string& display_name, const QVector3D& origin,
                       const QVector3D& linear, const QVector3D& angular,
                       const QColor& linear_color, const QColor& angular_color,
                       float linear_scale, float angular_scale, float width,
                       bool hide_small_values);

/**
 * @brief Draws a 6×6 pose covariance as an RViz CovarianceVisual.
 *
 * @param context Display context.
 * @param scene GL overlay fallback.
 * @param display_name Stable object-name prefix.
 * @param position Mean position (world).
 * @param pose_orientation Orientation of the pose (body → fixed).
 * @param frame_orientation Orientation of the parent frame (for ellipse axes).
 * @param covariance Row-major 6×6 covariance (geometry_msgs layout).
 * @param position_color Color for the position ellipsoid.
 * @param position_scale Scale for position covariance.
 * @param orientation_scale Scale for orientation covariance.
 * @param orientation_offset Offset applied to orientation visual.
 * @param visible Master visibility flag.
 * @return @c true if a backend accepted the draw.
 *
 * @see PoseWithCovarianceDisplay
 */
bool drawCovarianceOgreOrGl(
    common::DisplayContext* context, rendering::SceneOverlay& scene,
    const std::string& display_name, const QVector3D& position,
    const QQuaternion& pose_orientation, const QQuaternion& frame_orientation,
    const std::array<double, 36>& covariance, const QColor& position_color,
    float position_scale, float orientation_scale, float orientation_offset,
    bool visible);

}  // namespace display
}  // namespace autoviz
