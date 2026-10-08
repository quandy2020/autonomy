/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_render_window.hpp
 * @brief QWidget viewport backed by the Ogre render backend.
 *
 * Parallel to @ref RenderWindow for the Ogre path: hosts
 * @ref OgreRenderBackend, shares the same tool / view-controller interaction
 * model, and exposes @ref OgreSceneHost for persistent display geometry.
 *
 * @see RenderWindow
 * @see OgreRenderBackend
 * @see OgreSceneHost
 * @see ViewController
 */

#pragma once

#include <functional>
#include <string>

#include <QColor>
#include <QCursor>
#include <QWidget>

class QHideEvent;
class QPaintEngine;
class QShowEvent;

#include "autoviz/rendering/ogre_render_backend.hpp"
#include "autoviz/rendering/render_settings.hpp"
#include "autoviz/rendering/scene_overlay.hpp"
#include "autoviz/rendering/view_controller.hpp"
#include "autoviz/common/pick_handle.hpp"

namespace autoviz {
namespace common {
class ToolManager;
}

namespace rendering {

/**
 * @class OgreRenderWindow
 * @brief QWidget viewport backed by Ogre 1.x (the only Autoviz 3D path).
 *
 * Painting is driven by Qt @c paintEvent, which delegates to
 * @ref OgreRenderBackend::render().
 */
class OgreRenderWindow : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the Ogre-backed viewport.
   * @param parent Qt parent widget.
   */
  explicit OgreRenderWindow(QWidget* parent = nullptr);

  /** @brief Shuts down the Ogre backend. */
  ~OgreRenderWindow() override;

  /**
   * @brief Attaches the shared display scene overlay (non-owning).
   * @param overlay Overlay for immediate CPU geometry; may be @c nullptr.
   */
  void setSceneOverlay(SceneOverlay* overlay) { scene_overlay_ = overlay; }

  /**
   * @brief Sets the Ogre clear / background color.
   * @param color Clear color.
   */
  void setBackgroundColor(const QColor& color) {
    ogre_backend_.setBackgroundColor(color);
  }

  /**
   * @brief Hides the native Ogre surface without waiting for a Qt hide event.
   *
   * Used by Change panel so the GL window cannot cover the replacement dock.
   */
  void hideNativeSurface() { ogre_backend_.setWindowVisible(false); }

  /**
   * @brief Returns the owned camera controller.
   * @return Mutable @ref ViewController reference.
   */
  ViewController& viewController() { return view_controller_; }

  /**
   * @brief Const overload of @ref viewController().
   * @return Const @ref ViewController reference.
   */
  const ViewController& viewController() const { return view_controller_; }

  /**
   * @brief Attaches the global tool manager (non-owning).
   * @param tool_manager Tool manager; may be @c nullptr.
   */
  void setToolManager(common::ToolManager* tool_manager) {
    tool_manager_ = tool_manager;
  }

  /**
   * @brief Sets the per-viewport tool id.
   * @param tool_id Tool registry id.
   */
  void setViewportToolId(std::string tool_id) {
    viewport_tool_id_ = std::move(tool_id);
  }

  /**
   * @brief Returns the current per-viewport tool id.
   * @return Tool id string.
   */
  const std::string& viewportToolId() const { return viewport_tool_id_; }

  /**
   * @brief Applies the active tool cursor.
   * @param cursor Qt cursor.
   */
  void setToolCursor(const QCursor& cursor) {
    tool_cursor_ = cursor;
    setCursor(cursor);
  }

  /** @brief Refreshes the cursor from the tool manager. */
  void syncToolCursorFromManager();

  /**
   * @brief Advances FPS / tool timers.
   * @param delta_seconds Elapsed seconds.
   */
  void tick(float delta_seconds);

  /**
   * @brief Registers view-drag begin/end callbacks.
   * @param on_drag_started Drag start callback.
   * @param on_drag_ended Drag end callback.
   */
  void setViewInteractionCallbacks(std::function<void()> on_drag_started,
                                   std::function<void()> on_drag_ended);

  /**
   * @brief Called when this viewport should become the active dock.
   * @param on_activate Activation callback.
   */
  void setViewportActivationCallback(std::function<void()> on_activate) {
    on_viewport_activate_ = std::move(on_activate);
  }

  /**
   * @brief Depth-picks a world point via the Ogre/GL depth buffer.
   *
   * @param pixel_x Pixel X.
   * @param pixel_y Pixel Y.
   * @param view View matrix.
   * @param projection Projection matrix.
   * @param[out] world Filled on success.
   * @return @c true on a valid hit.
   */
  bool readDepthPick(int pixel_x, int pixel_y, const QMatrix4x4& view,
                     const QMatrix4x4& projection, QVector3D* world) const;

  /**
   * @brief Reads a pick handle at the given pixel (Ogre pick or FBO).
   *
   * @param pixel_x Pixel X.
   * @param pixel_y Pixel Y.
   * @return Pick handle or invalid handle.
   */
  common::PickHandle readPickHandleAt(int pixel_x, int pixel_y) const;

  /**
   * @brief Returns the persistent Ogre scene host for display uploads.
   * @return Host pointer (may be null before init).
   */
  OgreSceneHost* ogreSceneHost() { return ogre_backend_.ogreSceneHost(); }

  /**
   * @brief Const overload of @ref ogreSceneHost().
   * @return Const host pointer.
   */
  const OgreSceneHost* ogreSceneHost() const {
    return ogre_backend_.ogreSceneHost();
  }

 signals:
  /** @brief Emitted when the user starts dragging the camera. */
  void viewDragStarted();

  /** @brief Emitted while the camera is being dragged. */
  void viewDragUpdated();

  /** @brief Emitted when the camera drag ends. */
  void viewDragEnded();

  /** @brief Letter shortcut switched the active tool (g/p/…). */
  void toolShortcutTriggered();

 protected:
  void resizeEvent(QResizeEvent* event) override;
  void paintEvent(QPaintEvent* event) override;
  void showEvent(QShowEvent* event) override;
  void hideEvent(QHideEvent* event) override;
  /** Native Ogre GL surface — Qt must not use a software paint engine. */
  QPaintEngine* paintEngine() const override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;
  void enterEvent(QEnterEvent* event) override;

 private:
  SceneOverlay* scene_overlay_ = nullptr;
  ViewController view_controller_;
  common::ToolManager* tool_manager_ = nullptr;
  std::string viewport_tool_id_ = "Interact";
  QCursor tool_cursor_ = Qt::ArrowCursor;
  OgreRenderBackend ogre_backend_;
  std::function<void()> on_view_drag_started_;
  std::function<void()> on_view_drag_ended_;
  std::function<void()> on_viewport_activate_;
};

}  // namespace rendering
}  // namespace autoviz
