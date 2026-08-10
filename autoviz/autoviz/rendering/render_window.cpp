/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/render_window.hpp"

#include <QEnterEvent>
#include <QCursor>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QSize>
#include <QWheelEvent>
#include <cmath>

#include "autoviz/common/tool_manager.hpp"
#include "autoviz/platform/opengl_setup.hpp"
#include "autoviz/rendering/gpu_capabilities.hpp"
#include "autoviz/rendering/gpu_depth_pick.hpp"
#include "autoviz/rendering/viewport_mouse.hpp"

namespace autoviz {
namespace rendering {

RenderWindow::RenderWindow(QWidget* parent) : QOpenGLWidget(parent) {
  setMinimumSize(640, 480);
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
  setFormat(platform::defaultSurfaceFormat());
}

RenderWindow::~RenderWindow() {
  makeCurrent();
  pick_framebuffer_.destroy();
  grid_renderer_.shutdown();
  tool_overlay_.shutdown();
  // scene_overlay_ is owned by VisualizationManager and shared across
  // viewports — never shut it down from a viewport destructor.
  doneCurrent();
}

void RenderWindow::initializeGL() {
  GpuCapabilities::instance().probeFromOpenGL();
  grid_renderer_.initialize();
  if (scene_overlay_ != nullptr) {
    scene_overlay_->initialize();
  }
  tool_overlay_.initialize();
  syncRendererFramebufferSize();
}

QSize RenderWindow::framebufferSize() const {
  // QWidget::width/height are logical points; the GL FBO is in device pixels.
  // On Retina (dpr=2), using logical size for glViewport draws into the
  // bottom-left quarter of the panel.
  const qreal dpr = devicePixelRatioF();
  return QSize(
      std::max(1, static_cast<int>(std::lround(width() * dpr))),
      std::max(1, static_cast<int>(std::lround(height() * dpr))));
}

void RenderWindow::syncRendererFramebufferSize() {
  const QSize fb = framebufferSize();
  grid_renderer_.resize(fb.width(), fb.height());
  pick_framebuffer_.ensure(fb.width(), fb.height());
}

void RenderWindow::renderPickFramebuffer(const QMatrix4x4& view,
                                         const QMatrix4x4& projection) {
  const QSize fb = framebufferSize();
  pick_framebuffer_.renderPickPass(scene_overlay_, view, projection, fb.width(),
                                   fb.height());
}

bool RenderWindow::readDepthPick(int pixel_x, int pixel_y,
                                 const QMatrix4x4& view,
                                 const QMatrix4x4& projection,
                                 QVector3D* world) const {
  if (world == nullptr) {
    return false;
  }
  const QSize fb = framebufferSize();
  const qreal dpr = devicePixelRatioF();
  const int device_x = static_cast<int>(std::lround(pixel_x * dpr));
  const int device_y = static_cast<int>(std::lround(pixel_y * dpr));
  const_cast<RenderWindow*>(this)->makeCurrent();
  const GpuDepthPickResult pick = pickWorldPointFromDepthBuffer(
      device_x, device_y, fb.width(), fb.height(), view, projection);
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
  const QSize fb = framebufferSize();
  const qreal dpr = devicePixelRatioF();
  const int device_x = static_cast<int>(std::lround(pixel_x * dpr));
  const int device_y = static_cast<int>(std::lround(pixel_y * dpr));
  const_cast<RenderWindow*>(this)->makeCurrent();
  const common::PickHandle handle = pick_framebuffer_.readHandleAt(
      device_x, device_y, fb.width(), fb.height());
  const_cast<RenderWindow*>(this)->doneCurrent();
  return handle;
}

void RenderWindow::resizeGL(int /*width*/, int /*height*/) {
  // Ignore Qt's w/h: some versions pass logical size, others device pixels.
  // Always derive from widget size × devicePixelRatioF().
  syncRendererFramebufferSize();
}

void RenderWindow::paintGL() {
  // Keep viewport in sync when moving between screens with different DPR.
  syncRendererFramebufferSize();
  const float aspect =
      static_cast<float>(width()) / static_cast<float>(std::max(1, height()));
  const QMatrix4x4 view = view_controller_.viewMatrix();
  const QMatrix4x4 projection = view_controller_.projectionMatrix(aspect);
  grid_renderer_.render(view, projection);
  if (scene_overlay_ != nullptr) {
    scene_overlay_->render(view, projection);
    renderPickFramebuffer(view, projection);
  }
  // Measure (and similar) stay off the shared scene so Split panels are independent.
  if (tool_manager_ != nullptr && viewport_tool_id_ == "Measure") {
    tool_overlay_.clear();
    tool_manager_->drawTool("Measure", viewport_key_, tool_overlay_);
    tool_overlay_.render(view, projection);
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
  if (common::Tool* tool = tool_manager_->toolById(viewport_tool_id_)) {
    setToolCursor(tool->cursor());
  }
}

void RenderWindow::mousePressEvent(QMouseEvent* event) {
  if (on_viewport_activate_) {
    on_viewport_activate_();
  }
  if (tool_manager_ != nullptr) {
    if (tool_manager_->mousePressEvent(event, viewport_tool_id_)) {
      syncToolCursorFromManager();
      event->accept();
      return;
    }
    if (!tool_manager_->allowsViewportNavigation(viewport_tool_id_)) {
      syncToolCursorFromManager();
      event->accept();
      return;
    }
  }
  if (!view_controller_.handleMouseEvent(
          MakeViewportPressEvent(*event, width(), height()))) {
    event->ignore();
    return;
  }
  // Accept so Qt grabs the mouse; otherwise drag moves may not be delivered.
  event->accept();
  setFocus(Qt::MouseFocusReason);
  if (on_view_drag_started_) {
    on_view_drag_started_();
  }
  emit viewDragStarted();
  update();
}

void RenderWindow::mouseReleaseEvent(QMouseEvent* event) {
  if (tool_manager_ != nullptr) {
    if (tool_manager_->mouseReleaseEvent(event, viewport_tool_id_)) {
      view_controller_.handleMouseEvent(
          MakeViewportReleaseEvent(*event, width(), height()));
      if (on_view_drag_ended_) {
        on_view_drag_ended_();
      }
      emit viewDragEnded();
      event->accept();
      return;
    }
    if (!tool_manager_->allowsViewportNavigation(viewport_tool_id_)) {
      view_controller_.handleMouseEvent(
          MakeViewportReleaseEvent(*event, width(), height()));
      event->accept();
      return;
    }
  }
  view_controller_.handleMouseEvent(
      MakeViewportReleaseEvent(*event, width(), height()));
  if (on_view_drag_ended_) {
    on_view_drag_ended_();
  }
  emit viewDragEnded();
  event->accept();
  update();
}

void RenderWindow::mouseMoveEvent(QMouseEvent* event) {
  if (tool_manager_ != nullptr) {
    if (tool_manager_->mouseMoveEvent(event, viewport_tool_id_)) {
      syncToolCursorFromManager();
      event->accept();
      return;
    }
    if (!tool_manager_->allowsViewportNavigation(viewport_tool_id_)) {
      syncToolCursorFromManager();
      event->accept();
      return;
    }
  }
  if (!view_controller_.isViewDragging()) {
    event->ignore();
    return;
  }
  if (!view_controller_.handleMouseEvent(
          MakeViewportMoveEvent(*event, width(), height()))) {
    event->ignore();
    return;
  }
  event->accept();
  update();
  emit viewDragUpdated();
}

void RenderWindow::wheelEvent(QWheelEvent* event) {
  if (tool_manager_ != nullptr) {
    if (tool_manager_->wheelEvent(event, viewport_tool_id_)) {
      event->accept();
      return;
    }
    if (!tool_manager_->allowsViewportNavigation(viewport_tool_id_)) {
      event->accept();
      return;
    }
  }
  if (!view_controller_.handleMouseEvent(
          MakeViewportWheelEvent(*event, width(), height()))) {
    event->ignore();
    return;
  }
  event->accept();
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
