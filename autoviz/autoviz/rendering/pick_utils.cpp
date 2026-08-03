/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/pick_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "autoviz/common/tool.hpp"
#include "autoviz/rendering/scene_overlay.hpp"
#include "autoviz/rendering/view_controller.hpp"

namespace autoviz {
namespace rendering {
namespace {

constexpr float kEyeDepthEpsilon = 1e-4f;

float EyeDepth(const QMatrix4x4& view, const QVector3D& world) {
  const QVector4D eye = view * QVector4D(world, 1.f);
  return -eye.z();
}

bool ProjectToScreen(const QMatrix4x4& mvp, int viewport_width,
                     int viewport_height, const QVector3D& world,
                     float* out_x, float* out_y) {
  const QVector4D clip = mvp * QVector4D(world, 1.f);
  if (clip.w() <= 1e-4f) {
    return false;
  }
  const float ndc_x = clip.x() / clip.w();
  const float ndc_y = clip.y() / clip.w();
  *out_x = (ndc_x + 1.f) * 0.5f * static_cast<float>(viewport_width);
  *out_y = (1.f - ndc_y) * 0.5f * static_cast<float>(viewport_height);
  return true;
}

void ConsiderCandidate(const QVector3D& position,
                       const std::string& display_name,
                       const std::string& display_type, float screen_x,
                       float screen_y, float eye_depth, int pixel_x, int pixel_y,
                       float max_distance, PickResult* best) {
  const float dx = screen_x - static_cast<float>(pixel_x);
  const float dy = screen_y - static_cast<float>(pixel_y);
  const float distance = std::sqrt(dx * dx + dy * dy);
  if (distance > max_distance) {
    return;
  }
  if (!best->hit || distance < best->pixel_distance - 0.01f ||
      (std::abs(distance - best->pixel_distance) <= 0.01f &&
       eye_depth + kEyeDepthEpsilon < best->eye_depth)) {
    best->hit = true;
    best->position = position;
    best->display_name = display_name;
    best->display_type = display_type;
    best->pixel_distance = distance;
    best->eye_depth = eye_depth;
  }
}

void ConsiderGeometryVertex(const SceneOverlay::ColoredVertex& vertex,
                            const QMatrix4x4& view, const QMatrix4x4& mvp,
                            int viewport_width, int viewport_height, int pixel_x,
                            int pixel_y, float max_pixel_distance,
                            PickResult* best) {
  float sx = 0.f;
  float sy = 0.f;
  if (!ProjectToScreen(mvp, viewport_width, viewport_height, vertex.position(),
                       &sx, &sy)) {
    return;
  }
  ConsiderCandidate(vertex.position(), {}, {}, sx, sy,
                    EyeDepth(view, vertex.position()), pixel_x, pixel_y,
                    max_pixel_distance, best);
}

float ClosestRaySegmentDistance(const QVector3D& ray_origin,
                                const QVector3D& ray_direction,
                                const QVector3D& seg_a, const QVector3D& seg_b,
                                QVector3D* closest_on_segment) {
  const QVector3D u = ray_direction.normalized();
  const QVector3D v = seg_b - seg_a;
  const QVector3D w0 = ray_origin - seg_a;
  const float a = QVector3D::dotProduct(u, u);
  const float b = QVector3D::dotProduct(u, v);
  const float c = QVector3D::dotProduct(v, v);
  const float d = QVector3D::dotProduct(u, w0);
  const float e = QVector3D::dotProduct(v, w0);
  const float denom = a * c - b * b;
  float sc = 0.f;
  float tc = 0.f;
  if (denom < 1e-6f) {
    sc = 0.f;
    tc = (b > c ? d / b : e / c);
  } else {
    sc = (b * e - c * d) / denom;
    tc = (a * e - b * d) / denom;
  }
  tc = std::clamp(tc, 0.f, 1.f);
  const QVector3D point_on_ray = ray_origin + u * sc;
  const QVector3D point_on_segment = seg_a + v * tc;
  if (closest_on_segment != nullptr) {
    *closest_on_segment = point_on_segment;
  }
  return (point_on_ray - point_on_segment).length();
}

void ConsiderLineSegments(const SceneOverlay& overlay, const QMatrix4x4& view,
                          const QMatrix4x4& mvp, int viewport_width,
                          int viewport_height, int pixel_x, int pixel_y,
                          float max_pixel_distance, PickResult* best) {
  const QMatrix4x4 inverse = mvp.inverted();
  const float ndc_x =
      (2.f * static_cast<float>(pixel_x) / static_cast<float>(viewport_width)) -
      1.f;
  const float ndc_y =
      1.f -
      (2.f * static_cast<float>(pixel_y) / static_cast<float>(viewport_height));
  const QVector4D near_point =
      inverse * QVector4D(ndc_x, ndc_y, -1.f, 1.f);
  const QVector4D far_point = inverse * QVector4D(ndc_x, ndc_y, 1.f, 1.f);
  const QVector3D ray_origin =
      near_point.toVector3D() / std::max(near_point.w(), 1e-4f);
  const QVector3D ray_direction =
      (far_point.toVector3D() / std::max(far_point.w(), 1e-4f) - ray_origin);

  const auto& lines = overlay.lineVertices();
  for (std::size_t i = 0; i + 1 < lines.size(); i += 2) {
    QVector3D hit_on_segment;
    const float distance = ClosestRaySegmentDistance(
        ray_origin, ray_direction, lines[i].position(), lines[i + 1].position(),
        &hit_on_segment);
    if (distance > 0.25f) {
      continue;
    }
    float sx = 0.f;
    float sy = 0.f;
    if (!ProjectToScreen(mvp, viewport_width, viewport_height, hit_on_segment,
                         &sx, &sy)) {
      continue;
    }
    ConsiderCandidate(hit_on_segment, {}, {}, sx, sy,
                      EyeDepth(view, hit_on_segment), pixel_x, pixel_y,
                      max_pixel_distance, best);
  }
}

PickResult MatchPickMetadataNear(const SceneOverlay& overlay,
                                 const QVector3D& world) {
  PickResult result;
  result.hit = true;
  *result.mutable_position() = world;
  float best_dist_sq = std::numeric_limits<float>::max();
  for (const auto& sample : overlay.pickSamples()) {
    const float dist_sq = (sample.position - world).lengthSquared();
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      result.display_name = sample.display_name;
      result.display_type = sample.display_type;
    }
  }
  return result;
}

}  // namespace

PickResult pickNearestScenePoint(const SceneOverlay& overlay,
                                 const QMatrix4x4& view,
                                 const QMatrix4x4& projection, int viewport_width,
                                 int viewport_height, int pixel_x, int pixel_y,
                                 float max_pixel_distance) {
  PickResult best;
  if (viewport_width <= 0 || viewport_height <= 0) {
    return best;
  }
  const QMatrix4x4 mvp = projection * view;

  for (const auto& sample : overlay.pickSamples()) {
    float sx = 0.f;
    float sy = 0.f;
    if (!ProjectToScreen(mvp, viewport_width, viewport_height, sample.position(),
                         &sx, &sy)) {
      continue;
    }
    ConsiderCandidate(sample.position(), sample.display_name, sample.display_type,
                      sx, sy, EyeDepth(view, sample.position()), pixel_x, pixel_y,
                      max_pixel_distance, &best);
  }

  ConsiderLineSegments(overlay, view, mvp, viewport_width, viewport_height,
                       pixel_x, pixel_y, max_pixel_distance, &best);

  for (const auto& vertex : overlay.pointVertices()) {
    ConsiderGeometryVertex(vertex, view, mvp, viewport_width, viewport_height,
                           pixel_x, pixel_y, max_pixel_distance, &best);
  }
  for (const auto& vertex : overlay.triangleVertices()) {
    ConsiderGeometryVertex(vertex, view, mvp, viewport_width, viewport_height,
                           pixel_x, pixel_y, max_pixel_distance, &best);
  }

  return best;
}

PickResult pickScenePoint(
    const SceneOverlay& overlay, const QMatrix4x4& view,
    const QMatrix4x4& projection, int viewport_width, int viewport_height,
    int pixel_x, int pixel_y, bool gpu_picking_enabled,
    const std::function<bool(int x, int y, QVector3D* world)>& gpu_depth_pick,
    float max_pixel_distance) {
  if (gpu_picking_enabled && gpu_depth_pick) {
    QVector3D world;
    if (gpu_depth_pick(pixel_x, pixel_y, &world)) {
      PickResult result = MatchPickMetadataNear(overlay, world);
      result.used_gpu_depth = true;
      return result;
    }
  }
  return pickNearestScenePoint(overlay, view, projection, viewport_width,
                               viewport_height, pixel_x, pixel_y,
                               max_pixel_distance);
}

PickResult pickAtToolContext(const common::ToolContext& context, int pixel_x,
                             int pixel_y, float max_pixel_distance) {
  PickResult miss;
  if (context.view_controller == nullptr || context.scene_overlay == nullptr ||
      context.viewport_width <= 0 || context.viewport_height <= 0) {
    return miss;
  }
  const float aspect =
      static_cast<float>(context.viewport_width) /
      static_cast<float>(std::max(1, context.viewport_height));

  if (context.gpu_picking_enabled && context.gpu_pick_id_read &&
      context.pick_registry != nullptr) {
    const common::PickHandle handle =
        context.gpu_pick_id_read(pixel_x, pixel_y);
    if (handle != common::kInvalidPickHandle) {
      if (const common::PickRecord* record =
              context.pick_registry->lookup(handle)) {
        PickResult result;
        result.hit = true;
        *result.mutable_position() = record->position;
        result.display_name = record->display_name;
        result.display_type = record->display_type;
        result.pick_handle = handle;
        result.point_index = record->point_index;
        result.used_gpu_depth = true;
        if (context.handler_manager != nullptr) {
          if (common::SelectionHandler* handler =
                  context.handler_manager->lookup(handle)) {
            result.properties = handler->properties();
            context.handler_manager->notifySelected(handle);
          }
        }
        return result;
      }
    }
  }

  return pickScenePoint(
      *context.scene_overlay, context.view_controller->viewMatrix(),
      context.view_controller->projectionMatrix(aspect), context.viewport_width,
      context.viewport_height, pixel_x, pixel_y, context.gpu_picking_enabled,
      context.gpu_depth_pick, max_pixel_distance);
}

}  // namespace rendering
}  // namespace autoviz
