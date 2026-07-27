/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/screen_projection.hpp"

#include <cmath>

#include <QMatrix4x4>
#include <QVector4D>

#include "autoviz/common/display_context.hpp"

namespace autoviz {
namespace display {
namespace {

float EyeDepth(const QMatrix4x4& view, const QVector3D& world) {
  const QVector4D eye = view * QVector4D(world, 1.f);
  return -eye.z();
}

bool ProjectToScreen(const QMatrix4x4& mvp, int viewport_width, int viewport_height,
                     const QVector3D& world, float* out_x, float* out_y) {
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

bool BuildMvp(const common::DisplayContext* context, QMatrix4x4* out_mvp) {
  if (context == nullptr || out_mvp == nullptr || !context->has_view_matrices ||
      context->viewport_width <= 0 || context->viewport_height <= 0) {
    return false;
  }
  *out_mvp = context->projection_matrix * context->view_matrix;
  return true;
}

}  // namespace

bool projectWorldToScreen(const common::DisplayContext* context, const QVector3D& world,
                          float* out_x, float* out_y) {
  if (out_x == nullptr || out_y == nullptr) {
    return false;
  }
  QMatrix4x4 mvp;
  if (!BuildMvp(context, &mvp)) {
    return false;
  }
  return ProjectToScreen(mvp, context->viewport_width, context->viewport_height, world,
                         out_x, out_y);
}

bool unprojectScreenToWorld(const common::DisplayContext* context, float screen_x,
                              float screen_y, float eye_depth, QVector3D* out_world) {
  if (context == nullptr || out_world == nullptr || !context->has_view_matrices ||
      context->viewport_width <= 0 || context->viewport_height <= 0) {
    return false;
  }

  const QMatrix4x4& view = context->view_matrix;
  const QMatrix4x4 inv_mvp = (context->projection_matrix * view).inverted();

  const float ndc_x =
      (2.f * screen_x / static_cast<float>(context->viewport_width)) - 1.f;
  const float ndc_y =
      1.f - (2.f * screen_y / static_cast<float>(context->viewport_height));

  const QVector4D world_h = inv_mvp * QVector4D(ndc_x, ndc_y, 0.f, 1.f);
  if (std::abs(world_h.w()) <= 1e-6f) {
    return false;
  }
  *out_world = world_h.toVector3D() / world_h.w();
  (void)eye_depth;
  return true;
}

QVector3D applyScreenOffset(const common::DisplayContext* context, const QVector3D& anchor,
                            float offset_x, float offset_y) {
  float screen_x = 0.f;
  float screen_y = 0.f;
  if (context == nullptr || !context->has_view_matrices ||
      !projectWorldToScreen(context, anchor, &screen_x, &screen_y)) {
    return anchor + QVector3D(offset_x * 0.01f, offset_y * 0.01f, 0.f);
  }

  const float eye_depth =
      std::max(EyeDepth(context->view_matrix, anchor), 0.1f);
  screen_x += offset_x;
  screen_y += offset_y;

  QVector3D offset_world;
  if (unprojectScreenToWorld(context, screen_x, screen_y, eye_depth, &offset_world)) {
    return offset_world;
  }
  return anchor + QVector3D(offset_x * 0.01f, offset_y * 0.01f, 0.f);
}

}  // namespace display
}  // namespace autoviz
