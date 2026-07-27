/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QMatrix4x4>

#include "autoviz/common/pick_handle.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace rendering {

/** Off-screen pick-color FBO shared by OpenGL and Ogre GL backends. */
class GlPickFramebuffer {
 public:
  ~GlPickFramebuffer();

  void ensure(int width, int height);
  void destroy();

  void renderPickPass(SceneOverlay* overlay, const QMatrix4x4& view,
                      const QMatrix4x4& projection, int width, int height);

  common::PickHandle readHandleAt(int pixel_x, int pixel_y, int width,
                                  int height) const;

  bool valid() const { return fbo_ != 0; }

 private:
  unsigned fbo_ = 0;
  unsigned color_tex_ = 0;
  unsigned depth_rbo_ = 0;
  int width_ = 0;
  int height_ = 0;
};

}  // namespace rendering
}  // namespace autoviz
