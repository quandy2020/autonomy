/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/pick_handle.hpp"
#include "autoviz/rendering/ogre_render_window.hpp"

#include <algorithm>

#include <QEnterEvent>
#include <QHideEvent>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QMoveEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QResizeEvent>
#include <QShowEvent>
#include <QTimer>
#include <QWheelEvent>

#include "autoviz/common/tool_manager.hpp"
#include "autoviz/rendering/viewport_mouse.hpp"

namespace autoviz {
namespace rendering {
namespace {

constexpr QColor kViewportPlate(48, 48, 48);

}  // namespace

OgreRenderWindow::OgreRenderWindow(QWidget* parent)
    : QWidget(parent), ogre_backend_(this) {
  // Keep min size small so center-host splitters can resize beside other panels.
  setMinimumSize(80, 60);
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
  setAttribute(Qt::WA_NativeWindow);
  setAttribute(Qt::WA_OpaquePaintEvent);
  // Never WA_PaintOnScreen. That attribute is what turns a briefly-unbound
  // GLX drawable into a see-through X11 hole (desktop/IDE shows through).
  // paintEvent always lays an opaque plate, then Ogre presents on top.
  setOpaquePlateMode();
  hide();
}

OgreRenderWindow::~OgreRenderWindow() { ogre_backend_.shutdown(); }

QPaintEngine* OgreRenderWindow::paintEngine() const {
  return QWidget::paintEngine();
}

void OgreRenderWindow::setOpaquePlateMode() {
  setAttribute(Qt::WA_PaintOnScreen, false);
  setAttribute(Qt::WA_NoSystemBackground, false);
  setAutoFillBackground(true);
  QPalette pal = palette();
  pal.setColor(QPalette::Window, kViewportPlate);
  pal.setColor(QPalette::Base, kViewportPlate);
  setPalette(pal);
}

void OgreRenderWindow::setDirectGlMode() {
  // Kept for call-site compatibility. Direct PaintOnScreen mode is retired —
  // it is the X11 hole path after Panels→tile / mosaic drag.
  setOpaquePlateMode();
}

void OgreRenderWindow::coverNativeHole(bool want_visible) {
  if (!want_visible) {
    // Destroy GLX only. Never QWidget::hide() here — hiding the opaque plate
    // (or destroy()ing WA_NativeWindow) punches an X11 hole through Autoviz so
    // the IDE shows through the "3D View" on mosaic title-drag.
    ogre_backend_.setWindowVisible(false);
    setOpaquePlateMode();
    update();
    return;
  }
  if (width() < 2 || height() < 2) {
    coverNativeHole(false);
    return;
  }
  if (!ogre_backend_.isInitialized() && !ogre_backend_.initialize()) {
    coverNativeHole(false);
    return;
  }

  setOpaquePlateMode();
  if (!isVisible()) {
    show();
  }
  raise();
  (void)winId();

  if (!ogre_backend_.rebindExternalWindow()) {
    ogre_backend_.setWindowVisible(false);
    present_fail_backoff_ticks_ = std::max(present_fail_backoff_ticks_, 6);
    update();
    return;
  }
  ogre_backend_.setWindowVisible(true);

  const float aspect =
      static_cast<float>(width()) / static_cast<float>(std::max(1, height()));
  if (!ogre_backend_.render(/*show_grid=*/false, ReferenceGridSettings{},
                            scene_overlay_, view_controller_, aspect)) {
    // Mapped empty GLX child = X11 see-through hole. Destroy it; keep plate.
    ogre_backend_.setWindowVisible(false);
    present_fail_backoff_ticks_ = std::max(present_fail_backoff_ticks_, 6);
    update();
    return;
  }
  present_fail_backoff_ticks_ = 0;
  present_fail_streak_ = 0;
  update();
}

void OgreRenderWindow::tick(float delta_seconds) {
  if (!wants_native_surface_) {
    coverNativeHole(false);
    return;
  }
  if (present_fail_backoff_ticks_ > 0) {
    --present_fail_backoff_ticks_;
    ogre_backend_.setWindowVisible(false);
    setOpaquePlateMode();
    if (!isVisible()) {
      show();
    }
    update();
    return;
  }
  if (!isVisible()) {
    coverNativeHole(true);
  }
  if (!isVisible()) {
    return;
  }
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

void OgreRenderWindow::moveEvent(QMoveEvent* event) {
  QWidget::moveEvent(event);
  ogre_backend_.notifyWindowMovedOrResized();
}

void OgreRenderWindow::hideNativeSurface() {
  wants_native_surface_ = false;
  present_fail_backoff_ticks_ = 0;
  present_fail_streak_ = 0;
  coverNativeHole(false);
}

void OgreRenderWindow::showNativeSurface() {
  // Park clears WA_NativeWindow so dock reparent cannot map a see-through X
  // hole. Restore it before winId()/GLX rebind.
  setAttribute(Qt::WA_NativeWindow, true);
  setOpaquePlateMode();
  wants_native_surface_ = true;
  present_fail_backoff_ticks_ = 0;
  present_fail_streak_ = 0;
  coverNativeHole(true);
}

void OgreRenderWindow::invalidateGlSurface() {
  // Destroy Ogre child only — keep this QWidget visible as the opaque plate.
  ogre_backend_.setWindowVisible(false);
  setOpaquePlateMode();
  update();
}

void OgreRenderWindow::releaseNativeWindowForPark() {
  // Mosaic title-drag: unmap GLX, keep this QWidget visible as charcoal.
  // destroy()/hide() used to open a see-through hole over the desktop/IDE.
  wants_native_surface_ = false;
  present_fail_backoff_ticks_ = 0;
  present_fail_streak_ = 0;
  ogre_backend_.setWindowVisible(false);
  setOpaquePlateMode();
  if (!isVisible()) {
    show();
  }
  raise();
  update();
}

void OgreRenderWindow::paintEvent(QPaintEvent* event) {
  // Qt plate is always opaque (no WA_PaintOnScreen). Ogre presents into a
  // mapped child GLX on top when live; on failure the child is destroyed and
  // only this charcoal plate remains — never an X11 see-through hole.
  const QRect area = event != nullptr ? event->rect() : rect();

  if (wants_native_surface_ && width() >= 2 && height() >= 2) {
    const float aspect =
        static_cast<float>(width()) / static_cast<float>(std::max(1, height()));
    if (ogre_backend_.render(/*show_grid=*/false, ReferenceGridSettings{},
                             scene_overlay_, view_controller_, aspect)) {
      present_fail_streak_ = 0;
      present_fail_backoff_ticks_ = 0;
      // Keep an opaque plate under the GLX child (parent of the child window).
      QPainter painter(this);
      painter.fillRect(area, kViewportPlate);
      return;
    }
    if (view_controller_.isViewDragging()) {
      QPainter painter(this);
      painter.fillRect(area, kViewportPlate);
      return;
    }
    present_fail_streak_ = 0;
    present_fail_backoff_ticks_ = 8;
    ogre_backend_.setWindowVisible(false);
  }

  QPainter painter(this);
  painter.fillRect(area, kViewportPlate);
}

void OgreRenderWindow::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  setOpaquePlateMode();
  if (!wants_native_surface_) {
    QTimer::singleShot(0, this, [this]() {
      if (!wants_native_surface_) {
        ogre_backend_.setWindowVisible(false);
      }
    });
    return;
  }
  QTimer::singleShot(0, this, [this]() {
    if (!wants_native_surface_) {
      return;
    }
    coverNativeHole(true);
  });
}

void OgreRenderWindow::hideEvent(QHideEvent* event) {
  ogre_backend_.setWindowVisible(false);
  setOpaquePlateMode();
  QWidget::hideEvent(event);
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
      tool_manager_->forwardActiveToolKeyPress(event)) {
    emit toolShortcutTriggered();
    event->accept();
    return;
  }
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
