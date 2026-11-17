/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_render_backend.hpp
 * @brief Optional Ogre rendering backend hosted by a Qt @c QWidget.
 *
 * Owns the Ogre window/scene bootstrap for @ref OgreRenderWindow: initializes
 * @ref RenderSystem, renders grid + overlay + @ref OgreSceneHost content, and
 * exposes depth / pick queries after each frame.
 *
 * @see OgreRenderWindow
 * @see RenderSystem
 * @see OgreSceneHost
 * @see OgrePickRenderer
 */

#pragma once

#include <memory>

#include <QWidget>

#include <QColor>
#include <QImage>

#include "autoviz/common/pick_handle.hpp"
#include "autoviz/common/pick_registry.hpp"
#include "autoviz/rendering/gl_pick_framebuffer.hpp"
#include "autoviz/rendering/render_settings.hpp"
#include "autoviz/rendering/scene_overlay.hpp"
#include "autoviz/rendering/view_controller.hpp"

namespace autoviz {
namespace rendering {

class OgreSceneHost;

/**
 * @class OgreRenderBackend
 * @brief Optional Ogre rendering backend.
 *
 * Implementation details live in a private @c Impl (pimpl) so non-Ogre builds
 * can still compile the header shape. Non-copyable.
 *
 * ## Frame flow
 *
 * @ref render() updates the camera from @ref ViewController, draws optional
 * reference grid settings, uploads/draws @ref SceneOverlay CPU geometry, and
 * renders persistent @ref OgreSceneHost objects.
 */
class OgreRenderBackend {
 public:
  /**
   * @brief Binds the backend to a host Qt widget (window handle source).
   * @param host Non-null widget that will display the Ogre window.
   */
  explicit OgreRenderBackend(QWidget* host);

  /** @brief Calls @ref shutdown() and destroys @c Impl. */
  ~OgreRenderBackend();

  OgreRenderBackend(const OgreRenderBackend&) = delete;
  OgreRenderBackend& operator=(const OgreRenderBackend&) = delete;

  /**
   * @brief Creates Ogre Root / window / scene if needed.
   * @return @c true on success (or already initialized).
   */
  bool initialize();

  /**
   * @brief Whether @ref initialize() has created an Ogre render window.
   */
  bool isInitialized() const;

  /**
   * @brief Whether a live (non-destroyed) Ogre @c RenderWindow currently exists.
   *
   * False after park/hide destroy — used to detect transparent-hole panes that
   * should be closed when multiple 3D Views are open.
   */
  bool hasLiveRenderWindow() const;

  /**
   * @brief Resizes the Ogre render window / viewport.
   * @param width Width in pixels.
   * @param height Height in pixels.
   */
  void resize(int width, int height);

  /**
   * @brief Renders one frame.
   *
   * @param show_grid Whether to draw the built-in reference grid.
   * @param grid_settings Grid appearance when @p show_grid is true.
   * @param overlay Optional CPU overlay (lines/points/meshes); may be null.
   * @param view_controller Camera matrices source.
   * @param aspect_ratio Viewport aspect for projection.
   * @return @c false if the frame could not be presented (caller should not
   *         leave a @c WA_PaintOnScreen hole visible).
   */
  bool render(bool show_grid, const ReferenceGridSettings& grid_settings,
              SceneOverlay* overlay, const ViewController& view_controller,
              float aspect_ratio);

  /**
   * @brief Sets the clear / background color.
   * @param color Background color.
   */
  void setBackgroundColor(const QColor& color);

  /**
   * @brief Maps or destroys the Ogre child GLX @c RenderWindow.
   *
   * @p visible true remaps/rebinds for on-screen present. @p false unmaps and
   * destroys the child so a stale @c background_pixel=0 surface cannot hole
   * through the opaque Qt host after mosaic drag.
   *
   * @param visible @c true to show; @c false to destroy.
   */
  void setWindowVisible(bool visible);

  /**
   * @brief Optional CPU frame mirror (unused for present; kept for tooling).
   * @return Image in widget pixels; usually null.
   */
  const QImage& frameImage() const;

  /**
   * @brief Syncs the external GLX/HWND after the host top-level window moved.
   *
   * Child widgets do not get @c QMoveEvent when only the shell window moves;
   * callers must invoke this or the surface stays behind / goes transparent.
   */
  void notifyWindowMovedOrResized();

  /**
   * @brief Recreates the Ogre @c RenderWindow on the host's current @c winId().
   *
   * Mosaic drag/park reparents the Qt @c WA_NativeWindow and X11 often mints a
   * new drawable. @c windowMovedOrResized alone leaves a transparent pane
   * because the old external handle is dead. Scene/camera/overlays are kept.
   *
   * @return @c true when a live render window is bound to the current host.
   */
  bool rebindExternalWindow();

  /**
   * @brief Tears down Ogre objects owned by this backend.
   */
  void shutdown();

  /**
   * @brief Reads depth at a pixel after the last render.
   *
   * @note OpenGL context from the Ogre window must be current / valid.
   *
   * @param pixel_x Pixel X.
   * @param pixel_y Pixel Y.
   * @param viewport_width Viewport width.
   * @param viewport_height Viewport height.
   * @param view View matrix.
   * @param projection Projection matrix.
   * @param[out] world_out World hit on success.
   * @return @c true on a valid depth hit.
   */
  bool pickDepthAt(int pixel_x, int pixel_y, int viewport_width,
                   int viewport_height, const QMatrix4x4& view,
                   const QMatrix4x4& projection, QVector3D* world_out) const;

  /**
   * @brief Picks a @ref common::PickHandle at a pixel (Ogre pick renderer).
   *
   * @param pixel_x Pixel X.
   * @param pixel_y Pixel Y.
   * @param viewport_width Viewport width.
   * @param viewport_height Viewport height.
   * @param pick_registry Optional registry for handle decoding; may be null.
   * @return Pick handle or invalid handle.
   */
  common::PickHandle pickHandleAt(int pixel_x, int pixel_y, int viewport_width,
                                  int viewport_height,
                                  common::PickRegistry* pick_registry = nullptr) const;

  /**
   * @brief Returns the persistent scene host for display geometry.
   * @return Host pointer after successful @ref initialize(); else null.
   */
  OgreSceneHost* ogreSceneHost();

  /**
   * @brief Const overload of @ref ogreSceneHost().
   * @return Const host pointer.
   */
  const OgreSceneHost* ogreSceneHost() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_; /**< Opaque Ogre state. */
  QWidget* host_ = nullptr;    /**< Non-owning host widget. */
};

}  // namespace rendering
}  // namespace autoviz
