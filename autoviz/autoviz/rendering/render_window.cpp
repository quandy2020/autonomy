/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/render_window.hpp"

#include <QEnterEvent>
#include <QCursor>
#include <QEnterEvent>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>

#include "autoviz/common/tool_manager.hpp"
#include "autoviz/rendering/gpu_capabilities.hpp"
#include "autoviz/rendering/gpu_depth_pick.hpp"
#include "autoviz/rendering/viewport_mouse.hpp"

namespace autoviz {
namespace rendering {

RenderWindow::RenderWindow(QWidget* parent) : QOpenGLWidget(parent) {
  setMinimumSize(640, 480);
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
  QSurfaceFormat format;
  format.setVersion(3, 3);
  format.setProfile(QSurfaceFormat::CoreProfile);
  setFormat(format);
}

RenderWindow::~RenderWindow() {
  makeCurrent();
  pick_framebuffer_.destroy();
  grid_renderer_.shutdown();
  if (scene_overlay_ != nullptr) {
    scene_overlay_->shutdown();
  }
  doneCurrent();
}

void RenderWindow::initializeGL() {
  GpuCapabilities::instance().probeFromOpenGL();
  grid_renderer_.initialize();
  if (scene_overlay_ != nullptr) {
    scene_overlay_->initialize();
  }
}

void RenderWindow::renderPickFramebuffer(const QMatrix4x4& view,
                                         const QMatrix4x4& projection) {
  pick_framebuffer_.renderPickPass(scene_overlay_, view, projection, width(),
                                   height());
}

bool RenderWindow::readDepthPick(int pixel_x, int pixel_y,
                                 const QMatrix4x4& view,
                                 const QMatrix4x4& projection,
                                 QVector3D* world) const {
  if (world == nullptr) {
    return false;
  }
  const_cast<RenderWindow*>(this)->makeCurrent();
  const GpuDepthPickResult pick = pickWorldPointFromDepthBuffer(
      pixel_x, pixel_y, width(), height(), view, projection);
  const_cast<RenderWindow*>(this)->doneCurrent();
  if (!pick.hit) {
    return false;
  }
  *world = pick.position;
  return true;
}

common::PickHandle RenderWindow::readPickHandleAt(int pixel_x,
                                                  int pixel_y) const {
  if (!pick_framebuffer_.valid()) {
    return common::kInvalidPickHandle;
  }
  const_cast<RenderWindow*>(this)->makeCurrent();
  const common::PickHandle handle =
      pick_framebuffer_.readHandleAt(pixel_x, pixel_y, width(), height());
  const_cast<RenderWindow*>(this)->doneCurrent();
  return handle;
}

void RenderWindow::resizeGL(int width, int height) {
  grid_renderer_.resize(width, height);
  pick_framebuffer_.ensure(width, height);
}

void RenderWindow::paintGL() {
  const float aspect =
      static_cast<float>(width()) / static_cast<float>(std::max(1, height()));
  const QMatrix4x4 view = view_controller_.viewMatrix();
  const QMatrix4x4 projection = view_controller_.projectionMatrix(aspect);
  grid_renderer_.render(view, projection);
  if (scene_overlay_ != nullptr) {
    scene_overlay_->render(view, projection);
    renderPickFramebuffer(view, projection);
  }
}

void RenderWindow::tick(float delta_seconds) {
  view_controller_.tick(delta_seconds);
  update();
}

void RenderWindow::setViewInteractionCallbacks(
    std::function<void()> on_drag_started,
    std::function<void()> on_drag_ended) {
  on_view_drag_started_ = std::move(on_drag_started);
  on_view_drag_ended_ = std::move(on_drag_ended);
}

void RenderWindow::syncToolCursorFromManager() {
  if (tool_manager_ == nullptr) {
    return;
  }
  if (common::Tool* tool = tool_manager_->activeTool()) {
    setToolCursor(tool->cursor());
  }
}

void RenderWindow::mousePressEvent(QMouseEvent* event) {
  if (tool_manager_ != nullptr) {
    if (tool_manager_->mousePressEvent(event)) {
      syncToolCursorFromManager();
      return;
    }
    if (!tool_manager_->allowsViewportNavigation()) {
      syncToolCursorFromManager();
      return;
    }
  }
  if (!view_controller_.handleMouseEvent(
          MakeViewportPressEvent(*event, width(), height()))) {
    return;
  }
  if (on_view_drag_started_) {
    on_view_drag_started_();
  }
  emit viewDragStarted();
  update();
}

void RenderWindow::mouseReleaseEvent(QMouseEvent* event) {
  if (tool_manager_ != nullptr) {
    if (tool_manager_->mouseReleaseEvent(event)) {
      view_controller_.handleMouseEvent(
          MakeViewportReleaseEvent(*event, width(), height()));
      if (on_view_drag_ended_) {
        on_view_drag_ended_();
      }
      emit viewDragEnded();
      return;
    }
    if (!tool_manager_->allowsViewportNavigation()) {
      view_controller_.handleMouseEvent(
          MakeViewportReleaseEvent(*event, width(), height()));
      return;
    }
  }
  view_controller_.handleMouseEvent(
      MakeViewportReleaseEvent(*event, width(), height()));
  if (on_view_drag_ended_) {
    on_view_drag_ended_();
  }
  emit viewDragEnded();
  update();
}

void RenderWindow::mouseMoveEvent(QMouseEvent* event) {
  if (tool_manager_ != nullptr) {
    if (tool_manager_->mouseMoveEvent(event)) {
      syncToolCursorFromManager();
      return;
    }
    if (!tool_manager_->allowsViewportNavigation()) {
      syncToolCursorFromManager();
      return;
    }
  }
  if (!view_controller_.isViewDragging()) {
    return;
  }
  if (!view_controller_.handleMouseEvent(
          MakeViewportMoveEvent(*event, width(), height()))) {
    return;
  }
  update();
  emit viewDragUpdated();
}

void RenderWindow::wheelEvent(QWheelEvent* event) {
  if (tool_manager_ != nullptr) {
    if (tool_manager_->wheelEvent(event)) {
      return;
    }
    if (!tool_manager_->allowsViewportNavigation()) {
      return;
    }
  }
  if (!view_controller_.handleMouseEvent(
          MakeViewportWheelEvent(*event, width(), height()))) {
    return;
  }
  update();
  emit viewDragUpdated();
  emit viewDragEnded();
}

void RenderWindow::keyPressEvent(QKeyEvent* event) {
  if (view_controller_.handleKeyEvent(event->key(), true)) {
    emit viewDragUpdated();
    update();
    return;
  }
  view_controller_.setFpsKey(event->key(), true);
  update();
}

void RenderWindow::keyReleaseEvent(QKeyEvent* event) {
  view_controller_.setFpsKey(event->key(), false);
  update();
}

void RenderWindow::enterEvent(QEnterEvent* event) {
  QOpenGLWidget::enterEvent(event);
  setCursor(tool_cursor_);
}

}  // namespace rendering
}  // namespace autoviz
