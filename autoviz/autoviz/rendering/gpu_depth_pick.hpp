/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QMatrix4x4>
#include <QVector3D>

namespace autoviz {
namespace rendering {

struct GpuDepthPickResult {
  bool hit = false;
  QVector3D position;
};

/** Unproject a normalized depth sample [0,1] to world space. */
QVector3D unprojectDepthSample(const QMatrix4x4& inverse_view_projection,
                               float ndc_x, float ndc_y, float depth01);

/** Read depth buffer at pixel (requires current OpenGL context). */
GpuDepthPickResult pickWorldPointFromDepthBuffer(int pixel_x, int pixel_y,
                                                 int viewport_width,
                                                 int viewport_height,
                                                 const QMatrix4x4& view,
                                                 const QMatrix4x4& projection);

}  // namespace rendering
}  // namespace autoviz
