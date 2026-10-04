/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file render_window.hpp
 * @brief OpenGL @c QOpenGLWidget viewport with tools, picking, and camera.
 *
 * Primary non-Ogre 3D viewport: clears via @ref GridRenderer, draws
 * @ref SceneOverlay, routes mouse/keys through @ref ViewController and
 * @ref common::ToolManager, and maintains a @ref GlPickFramebuffer.
 *
 * @see OgreRenderWindow
 * @see ViewportWidget
 * @see ViewController
 * @see SceneOverlay
 */

#pragma once

#include <functional>
#include <string>

#include <QColor>
#include <QCursor>
#include <QOpenGLWidget>

#include "autoviz/rendering/grid_renderer.hpp"
#include "autoviz/rendering/gl_pick_framebuffer.hpp"
#include "autoviz/rendering/scene_overlay.hpp"
#include "autoviz/rendering/view_controller.hpp"
#include "autoviz/common/pick_handle.hpp"

namespace autoviz {
namespace common {
class ToolManager;
}

namespace rendering {

/**
 * @class RenderWindow
 * @brief OpenGL 3D viewport widget (HiDPI-aware FBO picking).
 *
 * ## Interaction
 *
 * - Mouse / wheel → active tool or @ref ViewController (Move Camera).
 * - Keys → tool shortcuts and FPS camera when applicable.
 * - @ref setViewportActivationCallback() runs on press so split docks activate.
 *
 * ## Picking
 *
 * After each paint, depth and pick-color buffers may be sampled via
 * @ref readDepthPick() / @ref readPickHandleAt() (caller must @c makeCurrent).
 *
 * @note Owns a local @c tool_overlay_ so Measure geometry does not leak across
 *       split panels; display geometry comes from the shared @c scene_overlay_.
 */
class RenderWindow : public QOpenGLWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the OpenGL viewport.
   * @param parent Qt parent widget.
   */
  explicit RenderWindow(QWidget* parent = nullptr);

  /** @brief Releases GL resources via @ref GridRenderer / pick FBO shutdown. */
  ~RenderWindow() override;

  /**
   * @brief Attaches the shared display scene overlay (non-owning).
   * @param overlay Overlay filled by displays each frame; may be @c nullptr.
   */
  void setSceneOverlay(SceneOverlay* overlay) { scene_overlay_ = overlay; }

  /**
   * @brief Sets the clear / background color for @ref GridRenderer.
   * @param color Background color.
   */
  void setBackgroundColor(const QColor& color) {
    grid_renderer_.setBackgroundColor(color);
  }

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
   * @brief Sets the per-viewport tool id (Select / Measure / Interact / …).
   *
   * Independent across split panels.
   *
   * @param tool_id Tool registry id string.
   */
  void setViewportToolId(std::string tool_id) {
    viewport_tool_id_ = std::move(tool_id);
  }

  /**
   * @brief Returns the current per-viewport tool id.
   * @return Tool id string (default @c "Interact").
   */
  const std::string& viewportToolId() const { return viewport_tool_id_; }

  /**
   * @brief Sets the dock objectName used as MeasureTool session key.
   * @param key Unique key per Split panel.
   */
  void setViewportKey(std::string key) { viewport_key_ = std::move(key); }

  /**
   * @brief Returns the viewport session key.
   * @return Key string (may be empty until set).
   */
  const std::string& viewportKey() const { return viewport_key_; }

  /**
   * @brief Applies the active tool cursor to this widget.
   * @param cursor Qt cursor for the tool.
   */
  void setToolCursor(const QCursor& cursor) {
    tool_cursor_ = cursor;
    setCursor(cursor);
  }

  /**
   * @brief Refreshes the cursor from @ref common::ToolManager for this viewport.
   */
  void syncToolCursorFromManager();

  /**
   * @brief Advances FPS / tool timers.
   * @param delta_seconds Elapsed seconds.
   */
  void tick(float delta_seconds);

  /**
   * @brief Registers callbacks for view-drag begin/end (camera interaction).
   * @param on_drag_started Invoked when a view drag starts.
   * @param on_drag_ended Invoked when a view drag ends.
   */
  void setViewInteractionCallbacks(std::function<void()> on_drag_started,
                                   std::function<void()> on_drag_ended);

  /**
   * @brief Called on mouse press before tool/view handling (activate this dock).
   * @param on_activate Activation callback.
   */
  void setViewportActivationCallback(std::function<void()> on_activate) {
    on_viewport_activate_ = std::move(on_activate);
  }

  /**
   * @brief Reads world position from the last frame's depth buffer.
   *
   * @note Requires @c makeCurrent(); uses last rendered depth.
   *
   * @param pixel_x Pixel X.
   * @param pixel_y Pixel Y.
   * @param view View matrix used for the frame.
   * @param projection Projection matrix used for the frame.
   * @param[out] world Filled on success.
   * @return @c true if a valid depth sample was unprojected.
   */
  bool readDepthPick(int pixel_x, int pixel_y, const QMatrix4x4& view,
                     const QMatrix4x4& projection, QVector3D* world) const;

  /**
   * @brief Reads the pick-color FBO from the last rendered frame.
   *
   * @param pixel_x Pixel X (device pixels).
   * @param pixel_y Pixel Y.
   * @return Pick handle or @ref common::kInvalidPickHandle.
   */
  common::PickHandle readPickHandleAt(int pixel_x, int pixel_y) const;

 signals:
  /** @brief Emitted when the user starts dragging the camera. */
  void viewDragStarted();

  /** @brief Emitted while the camera is being dragged. */
  void viewDragUpdated();

  /** @brief Emitted when the camera drag ends. */
  void viewDragEnded();

 protected:
  /** @brief Creates GL resources and initializes @ref GridRenderer. */
  void initializeGL() override;

  /**
   * @brief Handles resize; updates grid and pick FBO sizes.
   * @param width New width.
   * @param height New height.
   */
  void resizeGL(int width, int height) override;

  /** @brief Clears, draws overlay (+ tool overlay), and updates pick FBO. */
  void paintGL() override;

  void mousePressEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;
  void enterEvent(QEnterEvent* event) override;

 private:
  /**
   * @brief Renders pick geometry into @ref pick_framebuffer_.
   * @param view View matrix.
   * @param projection Projection matrix.
   */
  void renderPickFramebuffer(const QMatrix4x4& view, const QMatrix4x4& projection);

  /**
   * @brief OpenGL FBO size in device pixels (HiDPI-aware).
   * @return Framebuffer size.
   */
  QSize framebufferSize() const;

  /** @brief Syncs @ref GridRenderer and pick FBO to @ref framebufferSize(). */
  void syncRendererFramebufferSize();

  GridRenderer grid_renderer_;
  SceneOverlay* scene_overlay_ = nullptr;
  /** Local overlay for tools that must not share geometry across Split panels. */
  SceneOverlay tool_overlay_;
  ViewController view_controller_;
  common::ToolManager* tool_manager_ = nullptr;
  std::string viewport_tool_id_ = "Interact";
  std::string viewport_key_;
  QCursor tool_cursor_ = Qt::ArrowCursor;
  GlPickFramebuffer pick_framebuffer_;
  std::function<void()> on_view_drag_started_;
  std::function<void()> on_view_drag_ended_;
  std::function<void()> on_viewport_activate_;
};

}  // namespace rendering
}  // namespace autoviz
