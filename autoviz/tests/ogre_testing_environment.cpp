/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 ****************************************************************************/

#include "tests/ogre_testing_environment.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <memory>
#include <stdexcept>

#include <QGuiApplication>
#include <QOffscreenSurface>
#include <QOpenGLContext>

#include <OgreLogManager.h>

#include "autoviz/rendering/render_system.hpp"

#if defined(__linux__) && !defined(__APPLE__)
#include <GL/glx.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#endif

namespace autoviz {
namespace rendering {
namespace test {
namespace {

std::unique_ptr<QGuiApplication> g_qt_app;
std::unique_ptr<QOffscreenSurface> g_offscreen_surface;
std::unique_ptr<QOpenGLContext> g_gl_context;
std::unique_ptr<Ogre::LogManager> g_log_manager;
bool g_ogre_ready = false;

#if defined(__linux__) && !defined(__APPLE__)
Display* g_x11_display = nullptr;
GLXContext g_x11_context = nullptr;
XVisualInfo* g_x11_visual = nullptr;
Window g_x11_window = 0;

void EnsureX11GlxContext() {
  if (g_x11_window != 0) {
    glXMakeCurrent(g_x11_display, g_x11_window, g_x11_context);
    return;
  }
  g_x11_display = XOpenDisplay(nullptr);
  if (g_x11_display == nullptr) {
    throw std::runtime_error("Unable to open X11 display for Ogre tests.");
  }
  const int screen = DefaultScreen(g_x11_display);
  const int attrib_list[] = {GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE, 16,
                             GLX_STENCIL_SIZE, 8, None};
  g_x11_visual = glXChooseVisual(g_x11_display, screen, const_cast<int*>(attrib_list));
  if (g_x11_visual == nullptr) {
    throw std::runtime_error("Unable to create GLX visual for Ogre tests.");
  }
  g_x11_window = XCreateSimpleWindow(g_x11_display, RootWindow(g_x11_display, screen),
                                     0, 0, 1, 1, 0, 0, 0);
  g_x11_context = glXCreateContext(g_x11_display, g_x11_visual, nullptr, True);
  if (g_x11_context == nullptr) {
    throw std::runtime_error("Unable to create GLX context for Ogre tests.");
  }
  if (!glXMakeCurrent(g_x11_display, g_x11_window, g_x11_context)) {
    throw std::runtime_error("Unable to activate GLX context for Ogre tests.");
  }
  RenderSystem::setTestWindowHandle(static_cast<RenderSystem::WindowHandle>(g_x11_window));
}

void TeardownX11GlxContext() {
  if (g_x11_display == nullptr) {
    return;
  }
  if (g_x11_context != nullptr) {
    glXDestroyContext(g_x11_display, g_x11_context);
    g_x11_context = nullptr;
  }
  if (g_x11_window != 0) {
    XDestroyWindow(g_x11_display, g_x11_window);
    g_x11_window = 0;
  }
  if (g_x11_visual != nullptr) {
    XFree(g_x11_visual);
    g_x11_visual = nullptr;
  }
  XCloseDisplay(g_x11_display);
  g_x11_display = nullptr;
  RenderSystem::setTestWindowHandle(0);
}
#endif

void EnsureOffscreenGl() {
  if (QGuiApplication::instance() == nullptr) {
    static int argc = 1;
    static char arg0[] = "autoviz_ogre_test";
    static char* argv[] = {arg0, nullptr};
    g_qt_app = std::make_unique<QGuiApplication>(argc, argv);
  }
}

void BootstrapOgre(bool debug) {
  if (g_ogre_ready) {
    return;
  }
  EnsureOffscreenGl();
  RenderSystem::setSkipRenderWindow(false);
  if (Ogre::LogManager::getSingletonPtr() == nullptr) {
    g_log_manager = std::make_unique<Ogre::LogManager>();
    g_log_manager->createLog("", false, debug, true);
  }
  if (!RenderSystem::instance()->ensureInitialized()) {
    throw std::runtime_error("Autoviz RenderSystem failed to initialize.");
  }
  g_ogre_ready = true;
}

}  // namespace

void OgreGtestEnvironment::SetUp() {
  try {
    BootstrapOgre(false);
  } catch (const std::exception& ex) {
    FAIL() << "Ogre global test setup failed: " << ex.what();
  }
}

void OgreGtestEnvironment::TearDown() {
  RenderSystem::destroyInstance();
  g_log_manager.reset();
#if defined(__linux__) && !defined(__APPLE__)
  TeardownX11GlxContext();
#endif
  g_gl_context.reset();
  g_offscreen_surface.reset();
  g_qt_app.reset();
  g_ogre_ready = false;
}

void OgreTestingEnvironment::setUpOgreTestEnvironment(bool debug) {
  BootstrapOgre(debug);
}

int OgreTestingEnvironment::glslVersion() const {
  return RenderSystem::instance()->glslVersion();
}

OgreTestingEnvironment::~OgreTestingEnvironment() = default;

}  // namespace test
}  // namespace rendering
}  // namespace autoviz

#endif
