/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/gpu_capabilities.hpp"

#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOffscreenSurface>

#include <algorithm>
#include <cctype>

namespace autoviz {
namespace rendering {
namespace {

std::string ToLowerAscii(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

}  // namespace

GpuCapabilities& GpuCapabilities::instance() {
  static GpuCapabilities capabilities;
  return capabilities;
}

bool GpuCapabilities::IsSoftwareRenderer(const std::string& renderer) {
  if (renderer.empty()) {
    return true;
  }
  const std::string lower = ToLowerAscii(renderer);
  static const char* kSoftwarePatterns[] = {
      "llvmpipe",
      "softpipe",
      "swiftshader",
      "microsoft basic render driver",
      "soft rasterizer",
      "mesa offscreen",
      "swrast",
      "software rasterizer",
      "lavapipe",
  };
  for (const char* pattern : kSoftwarePatterns) {
    if (lower.find(pattern) != std::string::npos) {
      return true;
    }
  }
  return false;
}

void GpuCapabilities::probeFromRendererString(const std::string& renderer) {
  if (!renderer.empty()) {
    renderer_name_ = renderer;
    has_hardware_gpu_ = !IsSoftwareRenderer(renderer);
    probed_ = true;
    return;
  }
  if (!probed_) {
    has_hardware_gpu_ = false;
    probed_ = true;
  }
}

void GpuCapabilities::probeFromOpenGL() {
  QOpenGLContext* context = QOpenGLContext::currentContext();
  if (context == nullptr || !context->isValid()) {
    probeFromRendererString({});
    return;
  }
  QOpenGLFunctions* gl = context->functions();
  if (gl == nullptr) {
    probeFromRendererString({});
    return;
  }
  const char* vendor = reinterpret_cast<const char*>(gl->glGetString(0x1F00));
  const char* renderer = reinterpret_cast<const char*>(gl->glGetString(0x1F01));
  std::string combined;
  if (vendor != nullptr) {
    combined += vendor;
    combined += ' ';
  }
  if (renderer != nullptr) {
    combined += renderer;
  }
  probeFromRendererString(combined);
}

void GpuCapabilities::ensureProbed() {
  if (probed_) {
    return;
  }
  QOffscreenSurface surface;
  surface.create();
  QOpenGLContext context;
  if (!context.create()) {
    probeFromRendererString({});
    return;
  }
  if (!context.makeCurrent(&surface)) {
    probeFromRendererString({});
    return;
  }
  probeFromOpenGL();
  context.doneCurrent();
}

}  // namespace rendering
}  // namespace autoviz
