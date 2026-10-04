/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file render_system.hpp
 * @brief Singleton Ogre bootstrap (rviz_rendering::RenderSystem equivalent).
 *
 * Owns @c Ogre::Root, overlay system, optional headless window, GL version
 * detection, and resource/plugin setup used by all Autoviz Ogre viewports.
 *
 * @see OgreRenderBackend
 * @see ogreResourceDirectory()
 * @see OgreOgreLogging
 */

#pragma once

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

/**
 * @class RenderSystem
 * @brief Process-wide Ogre initialization and render-window factory.
 *
 * ## Lifecycle
 *
 * - @ref instance() creates the singleton on first use.
 * - @ref ensureInitialized() loads plugins, configures RS, resources.
 * - @ref makeRenderWindow() attaches to a native window handle.
 * - @ref destroyInstance() / @ref shutdown() tear down for tests / exit.
 *
 * ## Test hooks
 *
 * @ref setTestWindowHandle() / @ref setSkipRenderWindow() allow headless CI
 * without a real display.
 */
class RenderSystem {
 public:
  /** @brief Opaque native window handle type (platform HWND / XID / …). */
  using WindowHandle = size_t;

  /**
   * @brief Returns the singleton, creating it if needed.
   * @return Non-null singleton pointer.
   */
  static RenderSystem* instance();

  /**
   * @brief Destroys the singleton and releases Ogre Root.
   */
  static void destroyInstance();

  /**
   * @brief Optional X11/GLX or platform window for headless bootstrap (tests).
   * @param handle Native window handle.
   */
  static void setTestWindowHandle(WindowHandle handle);

  /**
   * @brief When @c true, skip creating a real render window during setup.
   * @param skip Skip flag.
   */
  static void setSkipRenderWindow(bool skip);

  /**
   * @brief Disables MSAA / anti-aliasing for subsequent window creation.
   */
  static void disableAntiAliasing();

  /**
   * @brief Forces a GL version (e.g. 320 for 3.2) before init.
   * @param version Encoded version (major*100 + minor*10 style as used by rviz).
   */
  static void forceGlVersion(int version);

  /**
   * @brief Disables stereo path probing.
   */
  static void forceNoStereo();

  /**
   * @brief Whether stereo rendering is supported after setup.
   * @return Stereo capability flag.
   */
  bool isStereoSupported() const { return stereo_supported_; }

  /**
   * @brief Idempotent full Ogre initialization.
   * @return @c true on success.
   */
  bool ensureInitialized();

  /**
   * @brief Shuts down Root / overlays / headless window without destroying the
   *        singleton pointer (call @ref destroyInstance() to free it).
   */
  void shutdown();

  /**
   * @brief Returns the Ogre Root instance.
   * @return Non-null after successful init.
   */
  Ogre::Root* ogreRoot();

  /**
   * @brief Returns the Ogre OverlaySystem (may be null if unused).
   * @return Overlay system pointer.
   */
  Ogre::OverlaySystem* overlaySystem();

  /**
   * @brief Detected OpenGL version code.
   * @return Version integer (e.g. 320).
   */
  int glVersion() const { return gl_version_; }

  /**
   * @brief Detected GLSL version code.
   * @return GLSL version integer (e.g. 150).
   */
  int glslVersion() const { return glsl_version_; }

  /**
   * @brief Creates an Ogre @c RenderWindow for a native window id.
   *
   * @param window_id Native parent window handle.
   * @param width Width in pixels.
   * @param height Height in pixels.
   * @param pixel_ratio Device pixel ratio (HiDPI); default 1.0.
   * @return New or existing Ogre render window.
   */
  Ogre::RenderWindow* makeRenderWindow(WindowHandle window_id, unsigned width,
                                       unsigned height, double pixel_ratio = 1.0);

  /**
   * @brief Attaches overlay system listeners to a scene manager.
   * @param scene_manager Target scene manager.
   */
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

