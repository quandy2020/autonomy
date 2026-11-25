/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/teleop/teleop_hud_overlay.hpp"

#include <QCloseEvent>
#include <QEvent>
#include <QGuiApplication>
#include <QKeyEvent>
#include <QList>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QScreen>
#include <QShowEvent>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QWindow>

namespace autoviz {
namespace teleop {
namespace {

constexpr int kMinSide = 156;
constexpr int kMaxSide = 280;
constexpr int kDefaultSide = 200;
constexpr double kMinOpacity = 0.35;
constexpr double kMaxOpacity = 1.0;

QPoint GlobalPos(const QMouseEvent* event) {
  return event->globalPosition().toPoint();
}

}  // namespace

TeleopHudOverlay::TeleopHudOverlay(QWidget* parent)
    : QWidget(parent, Qt::Tool | Qt::FramelessWindowHint |
                          Qt::NoDropShadowWindowHint) {
  setObjectName(QStringLiteral("TeleopHudOverlay"));
  setWindowTitle(tr("Teleop"));
  setAttribute(Qt::WA_TranslucentBackground, true);
  setAutoFillBackground(false);
  setMinimumSize(kMinSide, kMinSide);
  setMaximumSize(kMaxSide, kMaxSide);
  setFixedSize(kDefaultSide, kDefaultSide);
  setFocusPolicy(Qt::ClickFocus);
  setCursor(Qt::SizeAllCursor);
  setToolTip(tr("Drag to move · knob to drive · wheel: opacity · Esc: dock"));

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  host_ = new QWidget(this);
  host_->setAttribute(Qt::WA_TranslucentBackground, true);
  host_layout_ = new QVBoxLayout(host_);
  host_layout_->setContentsMargins(0, 0, 0, 0);
  host_layout_->setSpacing(0);
  root->addWidget(host_, 1);

  setOpacity(0.92);
}

void TeleopHudOverlay::setControl(QWidget* control) {
  if (control_ == control) {
    return;
  }
  if (control_ != nullptr) {
    control_->removeEventFilter(this);
    host_layout_->removeWidget(control_);
    control_->setParent(nullptr);
    control_ = nullptr;
  }
  control_ = control;
  if (control_ != nullptr) {
    host_layout_->addWidget(control_, 1);
    control_->installEventFilter(this);
    control_->show();
  }
}

QWidget* TeleopHudOverlay::takeControl() {
  QWidget* hosted = control_;
  if (hosted == nullptr) {
    return nullptr;
  }
  hosted->removeEventFilter(this);
  host_layout_->removeWidget(hosted);
  hosted->setParent(nullptr);
  control_ = nullptr;
  return hosted;
}

void TeleopHudOverlay::setOpacity(double opacity) {
  opacity_ = qBound(kMinOpacity, opacity, kMaxOpacity);
  setWindowOpacity(opacity_);
}

void TeleopHudOverlay::placeDefault(QWidget* relative) {
  setFixedSize(kDefaultSide, kDefaultSide);
  QRect anchor;
  if (relative != nullptr) {
    if (QWidget* top = relative->window()) {
      anchor = top->frameGeometry();
    }
  }
  if (!anchor.isValid()) {
    if (QScreen* screen = QGuiApplication::primaryScreen()) {
      anchor = screen->availableGeometry();
    }
  }
  if (!anchor.isValid()) {
    return;
  }
  move(anchor.left() + 36, anchor.bottom() - height() - 64);
}

bool TeleopHudOverlay::applyGeometryIfOnScreen(const QRect& geometry) {
  const int side = qMin(geometry.width(), geometry.height());
  if (side < kMinSide || side > kMaxSide) {
    return false;
  }
  const QRect square(geometry.topLeft(), QSize(side, side));
  if (!geometryOnScreen(square)) {
    return false;
  }
  setFixedSize(side, side);
  move(square.topLeft());
  return true;
}

bool TeleopHudOverlay::geometryOnScreen(const QRect& geometry) {
  const QRect inner = geometry.adjusted(16, 16, -16, -16);
  const QList<QScreen*> screens = QGuiApplication::screens();
  for (QScreen* screen : screens) {
    if (screen != nullptr && screen->availableGeometry().intersects(inner)) {
      return true;
    }
  }
  return false;
}

void TeleopHudOverlay::paintEvent(QPaintEvent* event) { Q_UNUSED(event); }

void TeleopHudOverlay::closeEvent(QCloseEvent* event) {
  event->ignore();
  emit closeRequested();
}

void TeleopHudOverlay::beginDrag(const QPoint& global_pos) {
  dragging_ = true;
  drag_offset_ = global_pos - frameGeometry().topLeft();
  grabMouse();
}

void TeleopHudOverlay::endDrag() {
  if (!dragging_) {
    return;
  }
  dragging_ = false;
  if (mouseGrabber() == this) {
    releaseMouse();
  }
  emit geometryChanged();
}

void TeleopHudOverlay::mousePressEvent(QMouseEvent* event) {
  if (event->button() != Qt::LeftButton && event->button() != Qt::RightButton) {
    event->ignore();
    return;
  }
  beginDrag(GlobalPos(event));
  event->accept();
}

void TeleopHudOverlay::mouseMoveEvent(QMouseEvent* event) {
  if (!dragging_) {
    QWidget::mouseMoveEvent(event);
    return;
  }
  move(GlobalPos(event) - drag_offset_);
  event->accept();
}

void TeleopHudOverlay::mouseReleaseEvent(QMouseEvent* event) {
  if (!dragging_) {
    QWidget::mouseReleaseEvent(event);
    return;
  }
  endDrag();
  event->accept();
}

bool TeleopHudOverlay::eventFilter(QObject* watched, QEvent* event) {
  if (watched == control_ || (control_ != nullptr && control_->isAncestorOf(
                                 qobject_cast<QWidget*>(watched)))) {
    if (event->type() == QEvent::MouseButtonPress) {
      const auto* mouse = static_cast<QMouseEvent*>(event);
      if (mouse->button() == Qt::RightButton) {
        beginDrag(GlobalPos(mouse));
        return true;
      }
    }
  }
  if (dragging_ && (event->type() == QEvent::MouseMove ||
                    event->type() == QEvent::MouseButtonRelease)) {
    if (event->type() == QEvent::MouseMove) {
      const auto* mouse = static_cast<QMouseEvent*>(event);
      move(GlobalPos(mouse) - drag_offset_);
    } else {
      endDrag();
    }
    return true;
  }
  return QWidget::eventFilter(watched, event);
}

void TeleopHudOverlay::wheelEvent(QWheelEvent* event) {
  const int delta = event->angleDelta().y();
  if (delta == 0) {
    return;
  }
  setOpacity(opacity_ + (delta > 0 ? 0.06 : -0.06));
  emit opacityChanged(opacity_);
  event->accept();
}

void TeleopHudOverlay::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_Escape) {
    emit closeRequested();
    event->accept();
    return;
  }
  QWidget::keyPressEvent(event);
}

void TeleopHudOverlay::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  // Transient-for the Autoviz main window so the compositor stacks this pad
  // with the app (above Autoviz panels, behind other programs).
  if (QWindow* mine = windowHandle()) {
    if (QWidget* owner = parentWidget()) {
      if (QWindow* theirs = owner->window()->windowHandle()) {
        mine->setTransientParent(theirs);
      }
    }
  }
  raise();
}

}  // namespace teleop
}  // namespace autoviz
