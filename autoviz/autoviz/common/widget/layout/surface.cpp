/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/widget/layout/surface.hpp"

#include <QEnterEvent>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>

#include "autoviz/ui/theme/glass.hpp"

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
  setCursor(interactive && isEnabled() ? Qt::PointingHandCursor
                                       : Qt::ArrowCursor);
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

void Surface::changeEvent(QEvent* event) {
  if (event->type() == QEvent::EnabledChange) {
    setCursor(interactive_ && isEnabled() ? Qt::PointingHandCursor
                                          : Qt::ArrowCursor);
    if (!isEnabled()) {
      pressed_ = false;
    }
    update();
  }
  QFrame::changeEvent(event);
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
  const ThemeTokens& tokens = GetThemeTokens(this, tone_);
  QColor fill = tokens.surface;
  if (elevation_ == Elevation::kRaised) {
    fill = tokens.surface_raised;
  } else if (elevation_ == Elevation::kOverlay) {
    fill = tokens.surface_material;
  }
  if (selected_) {
    fill = tokens.surface_selected;
  }
  if (!isEnabled()) {
    fill = tokens.surface_sunken;
  } else if (interactive_ && pressed_) {
    fill = tokens.surface_pressed;
  } else if (interactive_ && hovered_) {
    fill = tokens.surface_hovered;
  }

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  const qreal elevation_inset = elevation_ == Elevation::kFlat ? 0.5 : 1.5;
  const qreal shadow_offset = elevation_ == Elevation::kOverlay ? 2.0 : 1.0;
  QRectF surface_rect = QRectF(rect()).adjusted(
      elevation_inset, elevation_inset, -elevation_inset,
      -elevation_inset - (elevation_ == Elevation::kFlat ? 0.0 : 1.0));

  if (elevation_ != Elevation::kFlat) {
    QColor shadow = tokens.shadow_ambient;
    if (elevation_ == Elevation::kRaised) {
      shadow.setAlpha(qRound(shadow.alpha() * 0.65));
    }
    painter.setPen(Qt::NoPen);
    painter.setBrush(shadow);
    painter.drawRoundedRect(surface_rect.translated(0.0, shadow_offset),
                            tokens.radius_medium, tokens.radius_medium);
  }

  // Liquid Glass: soft frosted plate instead of opaque white cards.
  if (glass::ShellIsLiquid() && elevation_ == Elevation::kFlat &&
      !selected_) {
    const glass::ShellTokens shell = glass::Shell();
    QColor plate = shell.glass_fill;
    plate.setAlpha(qMax(200, plate.alpha()));
    painter.setBrush(plate);
    painter.setPen(QPen(shell.glass_rim, 1.0));
    painter.drawRoundedRect(surface_rect, shell.control_radius,
                            shell.control_radius);
    QColor cool = shell.glass_cool;
    cool.setAlpha(22);
    painter.setBrush(cool);
    painter.setPen(Qt::NoPen);
    painter.drawRoundedRect(surface_rect, shell.control_radius,
                            shell.control_radius);
  } else {
    painter.setBrush(fill);
    const QColor outline =
        !isEnabled() ? tokens.border_subtle
                     : (selected_ ? tokens.accent : tokens.border_subtle);
    painter.setPen(QPen(outline, selected_ ? 1.5 : 1.0));
    painter.drawRoundedRect(surface_rect, tokens.radius_medium,
                            tokens.radius_medium);
  }

  if (interactive_ && hasFocus()) {
    painter.setBrush(Qt::NoBrush);
    painter.setPen(
        QPen(tokens.focus_ring_outer, tokens.focus_ring_width + 2.0));
    painter.drawRoundedRect(surface_rect.adjusted(2.0, 2.0, -2.0, -2.0),
                            tokens.radius_medium - 2.0,
                            tokens.radius_medium - 2.0);
    painter.setPen(QPen(tokens.focus_ring, 1.5));
    painter.drawRoundedRect(surface_rect.adjusted(1.0, 1.0, -1.0, -1.0),
                            tokens.radius_medium - 1.0,
                            tokens.radius_medium - 1.0);
  }
}

}  // namespace autoviz::widget
