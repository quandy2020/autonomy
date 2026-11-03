/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/layout/surface.hpp"

#include <QEnterEvent>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>

namespace autoviz::widget {

Surface::Surface(QWidget* parent) : QFrame(parent) {
  setAttribute(Qt::WA_Hover);
  setAttribute(Qt::WA_StyledBackground, false);
  setFrameStyle(QFrame::NoFrame);
  setFocusPolicy(Qt::NoFocus);
}

void Surface::SetElevation(Elevation elevation) {
  if (elevation_ == elevation) return;
  elevation_ = elevation;
  update();
}

void Surface::SetInteractive(bool interactive) {
  if (interactive_ == interactive) return;
  interactive_ = interactive;
  setCursor(interactive ? Qt::PointingHandCursor : Qt::ArrowCursor);
  setFocusPolicy(interactive ? Qt::StrongFocus : Qt::NoFocus);
  if (!interactive) {
    pressed_ = false;
  }
  update();
}

void Surface::SetSelected(bool selected) {
  if (selected_ == selected) return;
  selected_ = selected;
  update();
}

void Surface::SetTone(style::Tone tone) {
  if (tone_ == tone) return;
  tone_ = tone;
  update();
}

void Surface::enterEvent(QEnterEvent* event) {
  hovered_ = true;
  update();
  QFrame::enterEvent(event);
}

void Surface::keyPressEvent(QKeyEvent* event) {
  const bool activates = event->key() == Qt::Key_Space ||
                         event->key() == Qt::Key_Return ||
                         event->key() == Qt::Key_Enter;
  if (interactive_ && activates && !event->isAutoRepeat()) {
    pressed_ = true;
    update();
    event->accept();
    return;
  }
  QFrame::keyPressEvent(event);
}

void Surface::keyReleaseEvent(QKeyEvent* event) {
  const bool activates = event->key() == Qt::Key_Space ||
                         event->key() == Qt::Key_Return ||
                         event->key() == Qt::Key_Enter;
  if (interactive_ && activates && pressed_ && !event->isAutoRepeat()) {
    pressed_ = false;
    update();
    event->accept();
    emit Clicked();
    return;
  }
  QFrame::keyReleaseEvent(event);
}

void Surface::leaveEvent(QEvent* event) {
  hovered_ = false;
  pressed_ = false;
  update();
  QFrame::leaveEvent(event);
}

void Surface::mousePressEvent(QMouseEvent* event) {
  if (interactive_ && event->button() == Qt::LeftButton) {
    pressed_ = true;
    setFocus(Qt::MouseFocusReason);
    update();
    event->accept();
    return;
  }
  QFrame::mousePressEvent(event);
}

void Surface::mouseReleaseEvent(QMouseEvent* event) {
  if (interactive_ && pressed_ && event->button() == Qt::LeftButton) {
    pressed_ = false;
    update();
    const bool clicked = rect().contains(event->position().toPoint());
    event->accept();
    if (clicked) {
      emit Clicked();
    }
    return;
  }
  QFrame::mouseReleaseEvent(event);
}

void Surface::paintEvent(QPaintEvent* event) {
  Q_UNUSED(event);
  const ThemeTokens& tokens = GetThemeTokens(tone_);
  QColor fill =
      elevation_ == Elevation::kFlat ? tokens.surface : tokens.surface_raised;
  if (interactive_ && hovered_) fill = tokens.surface_hovered;
  if (interactive_ && pressed_) fill = fill.darker(104);

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setBrush(fill);
  const bool emphasized = selected_ || (interactive_ && hasFocus());
  painter.setPen(QPen(emphasized ? tokens.focus_ring : tokens.border,
                      emphasized ? tokens.focus_ring_width : 1));
  const qreal inset = emphasized ? tokens.focus_ring_width * 0.5 : 0.5;
  painter.drawRoundedRect(QRectF(rect()).adjusted(inset, inset, -inset, -inset),
                          tokens.radius_medium, tokens.radius_medium);
}

}  // namespace autoviz::widget
