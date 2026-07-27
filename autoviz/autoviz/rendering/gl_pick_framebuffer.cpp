/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/gl_pick_framebuffer.hpp"

#include <algorithm>

#include <QOpenGLContext>
#include <QOpenGLExtraFunctions>
#include <QOpenGLFunctions>

namespace autoviz {
namespace rendering {
namespace {

constexpr unsigned kGlFramebuffer = 0x8D40;
constexpr unsigned kGlColorAttachment0 = 0x8CE0;
constexpr unsigned kGlDepthAttachment = 0x8D00;
constexpr unsigned kGlTexture2D = 0x0DE1;
constexpr unsigned kGlRgba = 0x1908;
constexpr unsigned kGlUnsignedByte = 0x1401;
constexpr unsigned kGlRenderbuffer = 0x8D41;
constexpr unsigned kGlDepthComponent24 = 0x81A6;
constexpr unsigned kGlFramebufferComplete = 0x8CD5;

}  // namespace

GlPickFramebuffer::~GlPickFramebuffer() { destroy(); }

void GlPickFramebuffer::ensure(int width, int height) {
  if (width <= 0 || height <= 0) {
    return;
  }
  if (fbo_ != 0 && width_ == width && height_ == height) {
    return;
  }
  destroy();
  QOpenGLExtraFunctions* gl = QOpenGLContext::currentContext()->extraFunctions();
  if (gl == nullptr) {
    return;
  }
  gl->glGenFramebuffers(1, &fbo_);
  gl->glBindFramebuffer(kGlFramebuffer, fbo_);
  gl->glGenTextures(1, &color_tex_);
  gl->glBindTexture(kGlTexture2D, color_tex_);
  gl->glTexImage2D(kGlTexture2D, 0, static_cast<int>(kGlRgba), width, height, 0,
                   kGlRgba, kGlUnsignedByte, nullptr);
  gl->glTexParameteri(kGlTexture2D, 0x2801, 0x2601);
  gl->glTexParameteri(kGlTexture2D, 0x2800, 0x2601);
  gl->glFramebufferTexture2D(kGlFramebuffer, kGlColorAttachment0, kGlTexture2D,
                             color_tex_, 0);
  gl->glGenRenderbuffers(1, &depth_rbo_);
  gl->glBindRenderbuffer(kGlRenderbuffer, depth_rbo_);
  gl->glRenderbufferStorage(kGlRenderbuffer, kGlDepthComponent24, width,
                            height);
  gl->glFramebufferRenderbuffer(kGlFramebuffer, kGlDepthAttachment,
                                kGlRenderbuffer, depth_rbo_);
  if (gl->glCheckFramebufferStatus(kGlFramebuffer) != kGlFramebufferComplete) {
    destroy();
    return;
  }
  gl->glBindFramebuffer(kGlFramebuffer, 0);
  width_ = width;
  height_ = height;
}

void GlPickFramebuffer::destroy() {
  QOpenGLExtraFunctions* gl =
      QOpenGLContext::currentContext() != nullptr
          ? QOpenGLContext::currentContext()->extraFunctions()
          : nullptr;
  if (gl != nullptr) {
    if (fbo_ != 0) {
      gl->glDeleteFramebuffers(1, &fbo_);
    }
    if (color_tex_ != 0) {
      gl->glDeleteTextures(1, &color_tex_);
    }
    if (depth_rbo_ != 0) {
      gl->glDeleteRenderbuffers(1, &depth_rbo_);
    }
  }
  fbo_ = 0;
  color_tex_ = 0;
  depth_rbo_ = 0;
  width_ = 0;
  height_ = 0;
}

void GlPickFramebuffer::renderPickPass(SceneOverlay* overlay,
                                       const QMatrix4x4& view,
                                       const QMatrix4x4& projection, int width,
                                       int height) {
  if (overlay == nullptr || !overlay->hasPickGeometry()) {
    return;
  }
  if (!overlay->isInitialized()) {
    overlay->initialize();
  }
  ensure(width, height);
  if (fbo_ == 0) {
    return;
  }
  QOpenGLFunctions* gl = QOpenGLContext::currentContext()->functions();
  QOpenGLExtraFunctions* gl_extra =
      QOpenGLContext::currentContext()->extraFunctions();
  gl_extra->glBindFramebuffer(kGlFramebuffer, fbo_);
  gl->glViewport(0, 0, width, height);
  gl->glClearColor(0.f, 0.f, 0.f, 1.f);
  gl->glClear(0x00004000 | 0x00000100);
  overlay->renderPickPass(view, projection);
  gl_extra->glBindFramebuffer(kGlFramebuffer, 0);
}

common::PickHandle GlPickFramebuffer::readHandleAt(int pixel_x, int pixel_y,
                                                   int width,
                                                   int height) const {
  if (fbo_ == 0 || width_ <= 0 || height_ <= 0) {
    return common::kInvalidPickHandle;
  }
  QOpenGLFunctions* gl = QOpenGLContext::currentContext()->functions();
  QOpenGLExtraFunctions* gl_extra =
      QOpenGLContext::currentContext()->extraFunctions();
  if (gl == nullptr || gl_extra == nullptr) {
    return common::kInvalidPickHandle;
  }
  gl_extra->glBindFramebuffer(kGlFramebuffer, fbo_);
  const int read_x = std::clamp(pixel_x, 0, width - 1);
  const int read_y = std::clamp(height - 1 - pixel_y, 0, height - 1);
  unsigned char rgba[4] = {0, 0, 0, 0};
  gl->glReadPixels(read_x, read_y, 1, 1, kGlRgba, kGlUnsignedByte, rgba);
  gl_extra->glBindFramebuffer(kGlFramebuffer, 0);
  return common::pickColorToHandle(rgba[0], rgba[1], rgba[2]);
}

}  // namespace rendering
}  // namespace autoviz
