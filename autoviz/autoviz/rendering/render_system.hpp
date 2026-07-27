/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <cstddef>
#include <cstdint>

#include <OgreRoot.h>

namespace Ogre {
class OverlaySystem;
class RenderWindow;
class SceneManager;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

/** Singleton Ogre bootstrap (rviz_rendering::RenderSystem equivalent). */
class RenderSystem {
 public:
  using WindowHandle = size_t;

  static RenderSystem* instance();
  static void destroyInstance();

  /** Optional X11/GLX or platform window for headless bootstrap (tests). */
  static void setTestWindowHandle(WindowHandle handle);
  static void setSkipRenderWindow(bool skip);

  static void disableAntiAliasing();
  static void forceGlVersion(int version);
  static void forceNoStereo();
  bool isStereoSupported() const { return stereo_supported_; }

  bool ensureInitialized();
  void shutdown();

  Ogre::Root* ogreRoot();
  Ogre::OverlaySystem* overlaySystem();

  int glVersion() const { return gl_version_; }
  int glslVersion() const { return glsl_version_; }

  Ogre::RenderWindow* makeRenderWindow(WindowHandle window_id, unsigned width,
                                       unsigned height, double pixel_ratio = 1.0);

  void prepareOverlays(Ogre::SceneManager* scene_manager);

 private:
  RenderSystem();
  ~RenderSystem();

  void loadOgrePlugins();
  void setupRenderSystem();
  void detectGlVersion();
  void setupResources();
  void ensureHeadlessRenderWindow();

  static RenderSystem* instance_;

  Ogre::Root* ogre_root_ = nullptr;
  Ogre::OverlaySystem* overlay_system_ = nullptr;
  Ogre::RenderWindow* headless_window_ = nullptr;
  static WindowHandle test_window_handle_;
  static bool skip_render_window_;
  bool initialized_ = false;
  int gl_version_ = 320;
  int glsl_version_ = 150;
  bool stereo_supported_ = false;
  static bool use_anti_aliasing_;
  static int force_gl_version_;
  static bool force_no_stereo_;
};

}  // namespace rendering
}  // namespace autoviz

#endif
