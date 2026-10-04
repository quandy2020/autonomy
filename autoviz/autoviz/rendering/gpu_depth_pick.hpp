/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file gpu_depth_pick.hpp
 * @brief Unproject OpenGL depth-buffer samples to world-space hit points.
 *
 * Used by Select / Interact tools when GPU picking is enabled: read depth at
 * a pixel after the last frame, then recover a 3D position via the inverse
 * view-projection matrix.
 *
 * @see pickWorldPointFromDepthBuffer()
 * @see pickScenePoint()
 * @see RenderWindow::readDepthPick()
 */

#pragma once

#include <QMatrix4x4>
#include <QVector3D>

namespace autoviz {
namespace rendering {

/**
 * @struct GpuDepthPickResult
 * @brief Result of a single GPU depth-buffer pick sample.
 */
struct GpuDepthPickResult {
  /** @brief @c true when depth was in range and unprojection succeeded. */
  bool hit = false;

  /** @brief World-space hit position when @ref hit is @c true. */
  QVector3D position;
};

/**
 * @brief Unprojects a normalized depth sample \[0,1\] to world space.
 *
 * @param inverse_view_projection Inverse of (@p projection × @p view).
 * @param ndc_x Normalized device X in \[-1, 1\].
 * @param ndc_y Normalized device Y in \[-1, 1\].
 * @param depth01 Depth from the GL depth buffer, remapped to \[0, 1\].
 * @return World-space position (homogeneous divide applied).
 */
QVector3D unprojectDepthSample(const QMatrix4x4& inverse_view_projection,
                               float ndc_x, float ndc_y, float depth01);

/**
 * @brief Reads the depth buffer at a pixel and unprojects to world space.
 *
 * @note Requires the OpenGL context that rendered the last frame to be current.
 *
 * @param pixel_x Pixel X (viewport / framebuffer space).
 * @param pixel_y Pixel Y.
 * @param viewport_width Viewport width in pixels.
 * @param viewport_height Viewport height in pixels.
 * @param view Camera view matrix used for the last draw.
 * @param projection Camera projection matrix used for the last draw.
 * @return @ref GpuDepthPickResult with @c hit set on success.
 *
 * @see RenderWindow::readDepthPick()
 * @see OgreRenderBackend::pickDepthAt()
 */
GpuDepthPickResult pickWorldPointFromDepthBuffer(int pixel_x, int pixel_y,
                                                 int viewport_width,
                                                 int viewport_height,
                                                 const QMatrix4x4& view,
                                                 const QMatrix4x4& projection);

}  // namespace rendering
}  // namespace autoviz
