/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/gpu_depth_pick.hpp"

#include <QOpenGLContext>
#include <QOpenGLFunctions>

#include <algorithm>
#include <cmath>

namespace autoviz {
namespace rendering {

QVector3D unprojectDepthSample(const QMatrix4x4& inverse_view_projection,
                               float ndc_x, float ndc_y, float depth01) {
  const float ndc_z = depth01 * 2.f - 1.f;
  const QVector4D clip(ndc_x, ndc_y, ndc_z, 1.f);
  QVector4D world = inverse_view_projection * clip;
  if (std::abs(world.w()) < 1e-6f) {
    return {};
  }
  world /= world.w();
  return world.toVector3D();
}

GpuDepthPickResult pickWorldPointFromDepthBuffer(int pixel_x, int pixel_y,
                                                 int viewport_width,
                                                 int viewport_height,
                                                 const QMatrix4x4& view,
                                                 const QMatrix4x4& projection) {
  GpuDepthPickResult result;
  if (viewport_width <= 0 || viewport_height <= 0) {
    return result;
  }
  QOpenGLContext* context = QOpenGLContext::currentContext();
  if (context == nullptr || !context->isValid()) {
    return result;
  }
  QOpenGLFunctions* gl = context->functions();
  if (gl == nullptr) {
    return result;
  }

  const int read_x = std::clamp(pixel_x, 0, viewport_width - 1);
  const int read_y = std::clamp(viewport_height - 1 - pixel_y, 0,
                                viewport_height - 1);

  float depth = 1.f;
  gl->glReadPixels(read_x, read_y, 1, 1, 0x1902, 0x1406, &depth);
  if (depth >= 0.9999f) {
    return result;
  }

  const float ndc_x =
      (2.f * static_cast<float>(read_x) / static_cast<float>(viewport_width)) -
      1.f;
  const float ndc_y =
      (2.f * static_cast<float>(read_y) / static_cast<float>(viewport_height)) -
      1.f;
  const QMatrix4x4 inverse = (projection * view).inverted();
  result.position =
      unprojectDepthSample(inverse, ndc_x, ndc_y, depth);
  result.hit = true;
  return result;
}

}  // namespace rendering
}  // namespace autoviz
