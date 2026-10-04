/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file gl_pick_framebuffer.hpp
 * @brief Off-screen pick-color FBO shared by OpenGL and Ogre GL backends.
 *
 * Renders @ref SceneOverlay::renderPickPass() into a color+depth FBO, then
 * samples the encoded @ref common::PickHandle at a pixel. Used when the
 * native Ogre pick path is unavailable or for the pure OpenGL @ref RenderWindow.
 *
 * @see SceneOverlay::renderPickPass()
 * @see RenderWindow::readPickHandleAt()
 * @see common::PickHandle
 */

#pragma once

#include <QMatrix4x4>

#include "autoviz/common/pick_handle.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace rendering {

/**
 * @class GlPickFramebuffer
 * @brief OpenGL framebuffer object for GPU pick-color selection.
 *
 * ## Lifecycle
 *
 * Call @ref ensure() with the current viewport size before @ref renderPickPass().
 * @ref destroy() (or the destructor) releases GL objects; the owning widget
 * must have a current GL context.
 *
 * ## Coordinate system
 *
 * @ref readHandleAt() expects pixel coordinates in the same space as the FBO
 * (typically device pixels / framebuffer size, origin top-left matching Qt).
 */
class GlPickFramebuffer {
 public:
  /** @brief Releases FBO / texture / RBO if still allocated. */
  ~GlPickFramebuffer();

  /**
   * @brief Allocates or resizes the pick FBO to @p width × @p height.
   *
   * No-op if dimensions match an already valid FBO. Requires a current GL
   * context.
   *
   * @param width Framebuffer width in pixels (must be > 0).
   * @param height Framebuffer height in pixels (must be > 0).
   */
  void ensure(int width, int height);

  /**
   * @brief Destroys GL objects and resets @ref valid() to @c false.
   *
   * Requires a current GL context when objects were previously created.
   */
  void destroy();

  /**
   * @brief Clears and draws the overlay pick pass into this FBO.
   *
   * Binds the FBO, sets viewport, invokes
   * @ref SceneOverlay::renderPickPass(), then unbinds.
   *
   * @param overlay Scene overlay with pick geometry / registry; may be null
   *        (no-op draw).
   * @param view Camera view matrix.
   * @param projection Camera projection matrix.
   * @param width Viewport / FBO width.
   * @param height Viewport / FBO height.
   */
  void renderPickPass(SceneOverlay* overlay, const QMatrix4x4& view,
                      const QMatrix4x4& projection, int width, int height);

  /**
   * @brief Samples the pick-color attachment at a pixel.
   *
   * @param pixel_x X in FBO coordinates.
   * @param pixel_y Y in FBO coordinates.
   * @param width FBO width (must match last @ref ensure()).
   * @param height FBO height.
   * @return Decoded handle, or @ref common::kInvalidPickHandle on miss / invalid FBO.
   */
  common::PickHandle readHandleAt(int pixel_x, int pixel_y, int width,
                                  int height) const;

  /**
   * @brief Whether the FBO has been successfully created.
   * @return @c true if @c fbo_ != 0.
   */
  bool valid() const { return fbo_ != 0; }

 private:
  unsigned fbo_ = 0;       /**< GL framebuffer object name. */
  unsigned color_tex_ = 0; /**< Color attachment (pick RGBA). */
  unsigned depth_rbo_ = 0; /**< Depth renderbuffer. */
  int width_ = 0;          /**< Allocated width. */
  int height_ = 0;         /**< Allocated height. */
};

}  // namespace rendering
}  // namespace autoviz
