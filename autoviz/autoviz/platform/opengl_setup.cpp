/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/platform/opengl_setup.hpp"

#include <QCoreApplication>
#include <QSurfaceFormat>

#ifdef AUTOVIZ_USE_QML_DRONE
#include <QQuickWindow>
#include <QSGRendererInterface>
#endif

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <string>

namespace autoviz {
namespace platform {
namespace {

bool envEnabled(const char* name) {
  const char* value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') {
    return false;
  }
  return std::string(value) != "0";
}

bool envDisabled(const char* name) {
  const char* value = std::getenv(name);
  return value != nullptr && (value[0] == '\0' || std::string(value) == "0");
}

bool runningInDocker() {
  if (std::getenv("container") != nullptr) {
    return true;
  }
  return access("/.dockerenv", F_OK) == 0;
}

bool shouldUseSoftwareOpenGL() {
  if (envEnabled("AUTOVIZ_USE_HARDWARE_GL")) {
    return false;
  }
  if (envDisabled("AUTOVIZ_SOFTWARE_GL") ||
      envDisabled("LIBGL_ALWAYS_SOFTWARE")) {
    return false;
  }
  if (envEnabled("AUTOVIZ_SOFTWARE_GL") ||
      envEnabled("LIBGL_ALWAYS_SOFTWARE")) {
    return true;
  }
  if (runningInDocker() && std::getenv("NVIDIA_VISIBLE_DEVICES") == nullptr) {
    return true;
  }
  return false;
}

void setSoftwareGlEnv() {
  if (!envEnabled("LIBGL_ALWAYS_SOFTWARE")) {
    setenv("LIBGL_ALWAYS_SOFTWARE", "1", 0);
  }
  if (!envEnabled("__GLX_VENDOR_LIBRARY_NAME")) {
    setenv("__GLX_VENDOR_LIBRARY_NAME", "mesa", 0);
  }
  if (!envEnabled("QT_OPENGL")) {
    setenv("QT_OPENGL", "software", 0);
  }
  if (!envEnabled("GALLIUM_DRIVER")) {
    setenv("GALLIUM_DRIVER", "llvmpipe", 0);
  }
  if (!envEnabled("MESA_GL_VERSION_OVERRIDE")) {
    setenv("MESA_GL_VERSION_OVERRIDE", "3.3", 0);
  }
  if (!envEnabled("MESA_GLSL_VERSION_OVERRIDE")) {
    setenv("MESA_GLSL_VERSION_OVERRIDE", "330", 0);
  }
  if (!envEnabled("LIBGL_DRI3_DISABLE")) {
    setenv("LIBGL_DRI3_DISABLE", "1", 0);
  }
}

QSurfaceFormat buildSurfaceFormat(bool software) {
  QSurfaceFormat format;
  format.setRenderableType(QSurfaceFormat::OpenGL);
  format.setRedBufferSize(8);
  format.setGreenBufferSize(8);
  format.setBlueBufferSize(8);
  format.setAlphaBufferSize(8);
  format.setDepthBufferSize(24);
  format.setStencilBufferSize(8);
  format.setSamples(0);
  format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
  format.setVersion(3, 3);
  // llvmpipe + Qt Quick3D are more reliable with compatibility profile.
  format.setProfile(software ? QSurfaceFormat::CompatibilityProfile
                             : QSurfaceFormat::CoreProfile);
  return format;
}

}  // namespace

__attribute__((constructor(101))) static void autovizEarlyOpenGLEnv() {
  if (shouldUseSoftwareOpenGL()) {
    setSoftwareGlEnv();
  }
}

bool usesSoftwareOpenGL() { return shouldUseSoftwareOpenGL(); }

QSurfaceFormat defaultSurfaceFormat() {
  return buildSurfaceFormat(shouldUseSoftwareOpenGL());
}

void configureOpenGLDefaults() {
  const bool software = shouldUseSoftwareOpenGL();
  if (software) {
    setSoftwareGlEnv();
    QCoreApplication::setAttribute(Qt::AA_UseSoftwareOpenGL);
  }
  QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
#ifdef AUTOVIZ_USE_QML_DRONE
  QQuickWindow::setGraphicsApi(QSGRendererInterface::OpenGL);
#endif
  const QSurfaceFormat format = buildSurfaceFormat(software);
  QSurfaceFormat::setDefaultFormat(format);
  if (envEnabled("AUTOVIZ_DEBUG_GL")) {
    std::fprintf(stderr,
                 "autoviz: GL setup software=%d version=%d.%d profile=%d\n",
                 software ? 1 : 0, format.majorVersion(), format.minorVersion(),
                 static_cast<int>(format.profile()));
  }
}

}  // namespace platform
}  // namespace autoviz
