/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/ogre_overlay_draw.hpp"

#include <QtMath>

#include <cmath>

#include "autoviz/common/display_context.hpp"
#include "autoviz/display/arrow_mesh_utils.hpp"
#include "autoviz/display/ogre_colored_points_draw.hpp"
#include "autoviz/display/ogre_mesh_draw.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

#ifdef AUTOVIZ_USE_OGRE
#include "autoviz/rendering/ogre_scene_host.hpp"
#endif

namespace autoviz {
namespace display {
namespace {

void syncOgreDisplayVisibility(common::DisplayContext* context,
                               const std::string& display_name) {
#ifdef AUTOVIZ_USE_OGRE
  if (context != nullptr && context->ogre_scene_host != nullptr &&
      context->active_display_visibility_bits != nullptr) {
    context->ogre_scene_host->setDisplayVisibilityBits(
        display_name, *context->active_display_visibility_bits);
  }
#endif
}

}  // namespace

bool drawLineSegmentsOgreOrGl(common::DisplayContext* context,
                              rendering::SceneOverlay& scene,
                              const std::string& display_name,
                              const std::vector<LineSegment3D>& segments) {
#ifdef AUTOVIZ_USE_OGRE
  if (context != nullptr && context->ogre_scene_host != nullptr) {
    std::vector<rendering::OgreColoredLineSegment> ogre_segments;
    ogre_segments.reserve(segments.size());
    for (const auto& segment : segments) {
      ogre_segments.push_back({segment.a, segment.b, segment.color});
    }
    syncOgreDisplayVisibility(context, display_name);
    context->ogre_scene_host->setDisplayLines(display_name, ogre_segments);
    return true;
  }
#endif

  for (const auto& segment : segments) {
    scene.addLine(segment.a, segment.b, segment.color);
  }
  return false;
}

bool drawArrowOgreOrGl(common::DisplayContext* context,
                       rendering::SceneOverlay& scene,
                       const std::string& display_name, const QVector3D& start,
                       const QVector3D& end, const QColor& color,
                       float head_fraction, float shaft_diameter,
                       float head_diameter) {
#ifdef AUTOVIZ_USE_OGRE
  if (context != nullptr && context->ogre_scene_host != nullptr) {
    std::vector<ColoredMeshInstance> meshes;
    appendSolidArrowMeshes(&meshes, start, end, color, head_fraction,
                           shaft_diameter, head_diameter);
    drawMeshesOgreOrGl(context, scene, display_name, meshes);
    return true;
  }
#endif

  const QVector3D delta = end - start;
  const float length = delta.length();
  if (length < 1e-4f) {
    return false;
  }
  const QVector3D direction = delta / length;
  QVector3D side = QVector3D::crossProduct(direction, QVector3D(0.f, 0.f, 1.f));
  if (side.lengthSquared() < 1e-6f) {
    side = QVector3D::crossProduct(direction, QVector3D(0.f, 1.f, 0.f));
  }
  side.normalize();
  const float head = length * head_fraction;
  scene.addLine(start, end, color);
  scene.addLine(end, end - direction * head + side * head * 0.35f, color);
  scene.addLine(end, end - direction * head - side * head * 0.35f, color);
  return false;
}

bool drawBillboardStripOgreOrGl(common::DisplayContext* context,
                                rendering::SceneOverlay& scene,
                                const std::string& display_name,
                                const std::vector<QVector3D>& points,
                                const QColor& color, float line_width) {
#ifdef AUTOVIZ_USE_OGRE
  if (context != nullptr && context->ogre_scene_host != nullptr &&
      line_width > 0.f) {
    syncOgreDisplayVisibility(context, display_name);
    context->ogre_scene_host->setDisplayBillboardStrip(display_name, points,
                                                       color, line_width);
    return true;
  }
#else
  (void)context;
  (void)display_name;
#endif

  if (points.size() < 2) {
    return false;
  }
  if (line_width > 0.f) {
    scene.addViewFacingPolylineStrip(points, line_width, color);
    return false;
  }
  for (std::size_t i = 1; i < points.size(); ++i) {
    scene.addLine(points[i - 1], points[i], color);
  }
  return false;
}

bool drawVectorArrowOgreOrGl(common::DisplayContext* context,
                             rendering::SceneOverlay& scene,
                             const std::string& display_name,
                             const QVector3D& origin, const QVector3D& vector,
                             const QColor& color, float min_length,
                             float scale, float shaft_diameter,
                             float head_diameter) {
  const float length = std::max(min_length, vector.length() * scale);
  if (length <= 1e-5f) {
    drawColoredPointsOgreOrGl(context, scene, display_name + "/origin",
                              "VectorArrow", 4.f, rendering::PointCloudStyle::kSquares,
                              {{origin, color}}, false);
    return true;
  }
  const QVector3D end = origin + vector.normalized() * length;
  return drawArrowOgreOrGl(context, scene, display_name, origin, end, color,
                           0.2f, shaft_diameter, head_diameter);
}

bool drawWrenchOgreOrGl(common::DisplayContext* context,
                        rendering::SceneOverlay& scene,
                        const std::string& display_name, const QVector3D& origin,
                        const QVector3D& force, const QVector3D& torque,
                        const QColor& force_color, const QColor& torque_color,
                        float force_scale, float torque_scale, float width) {
#ifdef AUTOVIZ_USE_OGRE
  if (context != nullptr && context->ogre_scene_host != nullptr) {
    syncOgreDisplayVisibility(context, display_name);
    context->ogre_scene_host->setDisplayWrench(
        display_name, origin, force, torque, force_color, torque_color,
        force_scale, torque_scale, width);
    return true;
  }
#else
  (void)width;
#endif

  drawVectorArrowOgreOrGl(context, scene, display_name + "/force", origin, force,
                          force_color, 0.f, force_scale);
  drawVectorArrowOgreOrGl(context, scene, display_name + "/torque", origin,
                          torque, torque_color, 0.f, torque_scale);
  return false;
}

bool drawScrewOgreOrGl(common::DisplayContext* context,
                       rendering::SceneOverlay& scene,
                       const std::string& display_name, const QVector3D& origin,
                       const QVector3D& linear, const QVector3D& angular,
                       const QColor& linear_color, const QColor& angular_color,
                       float linear_scale, float angular_scale, float width,
                       bool hide_small_values) {
#ifdef AUTOVIZ_USE_OGRE
  if (context != nullptr && context->ogre_scene_host != nullptr) {
    syncOgreDisplayVisibility(context, display_name);
    context->ogre_scene_host->setDisplayScrew(
        display_name, origin, linear, angular, linear_color, angular_color,
        linear_scale, angular_scale, width, hide_small_values);
    return true;
  }
#else
  (void)width;
  (void)hide_small_values;
#endif

  drawVectorArrowOgreOrGl(context, scene, display_name + "/linear", origin,
                          linear, linear_color, 0.f, linear_scale);
  drawVectorArrowOgreOrGl(context, scene, display_name + "/angular", origin,
                          angular, angular_color, 0.f, angular_scale);
  return false;
}

bool drawCovarianceOgreOrGl(
    common::DisplayContext* context, rendering::SceneOverlay& scene,
    const std::string& display_name, const QVector3D& position,
    const QQuaternion& pose_orientation, const QQuaternion& frame_orientation,
    const std::array<double, 36>& covariance, const QColor& position_color,
    float position_scale, float orientation_scale, float orientation_offset,
    bool visible) {
#ifdef AUTOVIZ_USE_OGRE
  if (context != nullptr && context->ogre_scene_host != nullptr) {
    syncOgreDisplayVisibility(context, display_name);
    context->ogre_scene_host->setDisplayCovariance(
        display_name, position, pose_orientation, frame_orientation, covariance,
        position_color, position_scale, orientation_scale, orientation_offset,
        visible);
    return true;
  }
#else
  (void)pose_orientation;
  (void)frame_orientation;
  (void)covariance;
  (void)orientation_scale;
  (void)orientation_offset;
  (void)visible;
#endif

  if (!visible) {
    return false;
  }
  const float cov_x =
      covariance[0] > 0.0 ? static_cast<float>(std::sqrt(covariance[0])) : 0.f;
  const float cov_y =
      covariance[7] > 0.0 ? static_cast<float>(std::sqrt(covariance[7])) : 0.f;
  if (cov_x > 1e-4f || cov_y > 1e-4f) {
    scene.addEllipsoidWireframe(position, cov_x * position_scale,
                                cov_y * position_scale, 0.02f, position_color);
  }
  return false;
}

}  // namespace display
}  // namespace autoviz
