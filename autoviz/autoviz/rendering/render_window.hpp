/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

class RenderWindow : public QOpenGLWidget {
  Q_OBJECT

 public:
  explicit RenderWindow(QWidget* parent = nullptr);
  ~RenderWindow() override;

  void setSceneOverlay(SceneOverlay* overlay) { scene_overlay_ = overlay; }
  void setGridVisible(bool visible) { grid_renderer_.setVisible(visible); }
  void setReferenceGridSettings(const ReferenceGridSettings& settings) {
    grid_renderer_.setReferenceGridSettings(settings);
  }
  void setBackgroundColor(const QColor& color) {
    grid_renderer_.setBackgroundColor(color);
  }

  ViewController& viewController() { return view_controller_; }
  const ViewController& viewController() const { return view_controller_; }

  void setToolManager(common::ToolManager* tool_manager) {
    tool_manager_ = tool_manager;
  }
  /** Per-viewport tool (Select/Measure/Interact); independent across splits. */
  void setViewportToolId(std::string tool_id) {
    viewport_tool_id_ = std::move(tool_id);
  }
  const std::string& viewportToolId() const { return viewport_tool_id_; }
  /** Dock objectName — keys MeasureTool session state per Split panel. */
  void setViewportKey(std::string key) { viewport_key_ = std::move(key); }
  const std::string& viewportKey() const { return viewport_key_; }
  void setToolCursor(const QCursor& cursor) {
    tool_cursor_ = cursor;
    setCursor(cursor);
  }

  void syncToolCursorFromManager();

  void tick(float delta_seconds);

  void setViewInteractionCallbacks(std::function<void()> on_drag_started,
                                   std::function<void()> on_drag_ended);
  /** Called on mouse press before tool/view handling (activate this dock). */
  void setViewportActivationCallback(std::function<void()> on_activate) {
    on_viewport_activate_ = std::move(on_activate);
  }

  /** Requires makeCurrent; reads last rendered depth buffer. */
  bool readDepthPick(int pixel_x, int pixel_y, const QMatrix4x4& view,
                     const QMatrix4x4& projection, QVector3D* world) const;
  /** Reads pick-color FBO from the last rendered frame. */
  common::PickHandle readPickHandleAt(int pixel_x, int pixel_y) const;

 signals:
  void viewDragStarted();
  void viewDragUpdated();
  void viewDragEnded();

 protected:
  void initializeGL() override;
  void resizeGL(int width, int height) override;
  void paintGL() override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;
  void enterEvent(QEnterEvent* event) override;

 private:
  void renderPickFramebuffer(const QMatrix4x4& view, const QMatrix4x4& projection);
  /** OpenGL FBO size in device pixels (HiDPI-aware). */
  QSize framebufferSize() const;
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
