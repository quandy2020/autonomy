/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/pick_handle.hpp"
#include "autoviz/rendering/ogre_render_window.hpp"

#include <QEnterEvent>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QWheelEvent>

#include "autoviz/common/tool_manager.hpp"
#include "autoviz/rendering/viewport_mouse.hpp"

namespace autoviz {
namespace rendering {

OgreRenderWindow::OgreRenderWindow(QWidget* parent)
    : QWidget(parent), ogre_backend_(this) {
  setMinimumSize(640, 480);
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
  setAttribute(Qt::WA_NativeWindow);
  setAttribute(Qt::WA_PaintOnScreen);
  setAttribute(Qt::WA_OpaquePaintEvent);
}

OgreRenderWindow::~OgreRenderWindow() { ogre_backend_.shutdown(); }

void OgreRenderWindow::tick(float delta_seconds) {
  view_controller_.tick(delta_seconds);
  update();
}

void OgreRenderWindow::setViewInteractionCallbacks(
    std::function<void()> on_drag_started,
    std::function<void()> on_drag_ended) {
  on_view_drag_started_ = std::move(on_drag_started);
  on_view_drag_ended_ = std::move(on_drag_ended);
}

void OgreRenderWindow::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  ogre_backend_.resize(event->size().width(), event->size().height());
}

void OgreRenderWindow::paintEvent(QPaintEvent* /*event*/) {
  const float aspect =
      static_cast<float>(width()) / static_cast<float>(std::max(1, height()));
  ogre_backend_.render(grid_renderer_.visible(), reference_grid_settings_,
                       scene_overlay_, view_controller_, aspect);
}

bool OgreRenderWindow::readDepthPick(int pixel_x, int pixel_y,
                                     const QMatrix4x4& view,
                                     const QMatrix4x4& projection,
                                     QVector3D* world) const {
  if (world == nullptr) {
    return false;
  }
  return ogre_backend_.pickDepthAt(pixel_x, pixel_y, width(), height(), view,
                                   projection, world);
}

common::PickHandle OgreRenderWindow::readPickHandleAt(int pixel_x,
                                                      int pixel_y) const {
  common::PickRegistry* registry =
      scene_overlay_ != nullptr ? scene_overlay_->pickRegistry() : nullptr;
  return ogre_backend_.pickHandleAt(pixel_x, pixel_y, width(), height(),
                                    registry);
}

void OgreRenderWindow::syncToolCursorFromManager() {
  if (tool_manager_ == nullptr) {
    return;
  }
  if (common::Tool* tool = tool_manager_->toolById(viewport_tool_id_)) {
    setToolCursor(tool->cursor());
  }
}

void OgreRenderWindow::mousePressEvent(QMouseEvent* event) {
  if (on_viewport_activate_) {
    on_viewport_activate_();
  }
  if (tool_manager_ != nullptr) {
    if (tool_manager_->mousePressEvent(event, viewport_tool_id_)) {
      syncToolCursorFromManager();
      event->accept();
      update();
      return;
    }
    if (!tool_manager_->allowsViewportNavigation(viewport_tool_id_)) {
      syncToolCursorFromManager();
      event->accept();
      update();
      return;
    }
  }
  if (!view_controller_.handleMouseEvent(
          MakeViewportPressEvent(*event, width(), height()))) {
    event->ignore();
    return;
  }
  event->accept();
  setFocus(Qt::MouseFocusReason);
  if (on_view_drag_started_) {
    on_view_drag_started_();
  }
  emit viewDragStarted();
  update();
}

void OgreRenderWindow::mouseReleaseEvent(QMouseEvent* event) {
  if (tool_manager_ != nullptr) {
    if (tool_manager_->mouseReleaseEvent(event, viewport_tool_id_)) {
      view_controller_.handleMouseEvent(
          MakeViewportReleaseEvent(*event, width(), height()));
      if (on_view_drag_ended_) {
        on_view_drag_ended_();
      }
      emit viewDragEnded();
      event->accept();
      update();
      return;
    }
    if (!tool_manager_->allowsViewportNavigation(viewport_tool_id_)) {
      view_controller_.handleMouseEvent(
          MakeViewportReleaseEvent(*event, width(), height()));
      event->accept();
      update();
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

void OgreRenderWindow::mouseMoveEvent(QMouseEvent* event) {
  if (tool_manager_ != nullptr) {
    if (tool_manager_->mouseMoveEvent(event, viewport_tool_id_)) {
      syncToolCursorFromManager();
      event->accept();
      update();
      return;
    }
    if (!tool_manager_->allowsViewportNavigation(viewport_tool_id_)) {
      syncToolCursorFromManager();
      event->accept();
      update();
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

void OgreRenderWindow::wheelEvent(QWheelEvent* event) {
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

void OgreRenderWindow::keyPressEvent(QKeyEvent* event) {
  if (tool_manager_ != nullptr &&
      tool_manager_->handleShortcutKey(event->key())) {
    emit toolShortcutTriggered();
    event->accept();
    return;
  }
  if (view_controller_.handleKeyEvent(event->key(), true)) {
    emit viewDragUpdated();
    update();
    return;
  }
  view_controller_.setFpsKey(event->key(), true);
  update();
}

void OgreRenderWindow::keyReleaseEvent(QKeyEvent* event) {
  view_controller_.setFpsKey(event->key(), false);
  update();
}

void OgreRenderWindow::enterEvent(QEnterEvent* event) {
  QWidget::enterEvent(event);
  setCursor(tool_cursor_);
}

}  // namespace rendering
}  // namespace autoviz
