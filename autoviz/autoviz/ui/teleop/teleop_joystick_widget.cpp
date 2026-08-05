/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/teleop/teleop_joystick_widget.hpp"

#include <algorithm>
#include <cmath>

#include <QFontMetrics>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QResizeEvent>

namespace autoviz {
namespace teleop {
namespace {

constexpr double kKnobRadiusRatio = 0.22;
constexpr double kDeadzoneRatio = 0.08;

}  // namespace

TeleopJoystickWidget::TeleopJoystickWidget(const QString& label, QWidget* parent)
    : label_(label), QWidget(parent) {
  setMinimumSize(148, 168);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setMouseTracking(true);
  setFocusPolicy(Qt::StrongFocus);
}

QPointF TeleopJoystickWidget::normalizedValue() const { return knob_; }

void TeleopJoystickWidget::reset() {
  resetVisual();
  emit valueChanged(0.0, 0.0);
  emit released();
}

void TeleopJoystickWidget::resetVisual() {
  dragging_ = false;
  if (knob_.isNull() && !isVisible()) {
    return;
  }
  knob_ = QPointF(0.0, 0.0);
  if (mouseGrabber() == this) {
    releaseMouse();
  }
  update();
}

QRectF TeleopJoystickWidget::outerRect() const {
  const QFontMetrics metrics(font());
  const int label_h = metrics.height() + 8;
  const int side = std::min(width(), height() - label_h);
  const double x = (width() - side) * 0.5;
  const double y = (height() - label_h - side) * 0.5 + 4.0;
  return QRectF(x, y, side, side);
}

QPointF TeleopJoystickWidget::center() const {
  const QRectF outer = outerRect();
  return outer.center();
}

double TeleopJoystickWidget::radius() const { return outerRect().width() * 0.5; }

QPointF TeleopJoystickWidget::clampToUnitDisk(const QPointF& vector) const {
  const double length = std::hypot(vector.x(), vector.y());
  if (length <= 1.0 || length <= 1e-6) {
    return vector;
  }
  return vector / length;
}

void TeleopJoystickWidget::updateFromPosition(const QPointF& local_pos) {
  const QPointF center_pt = center();
  const double max_radius = radius() * (1.0 - kKnobRadiusRatio - 0.04);
  QPointF delta(local_pos.x() - center_pt.x(), local_pos.y() - center_pt.y());
  if (max_radius > 1e-3) {
    delta /= max_radius;
  }
  const QPointF clamped = clampToUnitDisk(delta);
  if (std::hypot(clamped.x() - knob_.x(), clamped.y() - knob_.y()) < 1e-4) {
    return;
  }
  knob_ = clamped;
  update();
  emit valueChanged(knob_.x(), knob_.y());
}

void TeleopJoystickWidget::paintEvent(QPaintEvent* /*event*/) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);

  painter.fillRect(rect(), QColor(28, 28, 30));

  const QRectF outer = outerRect();
  const QPointF center_pt = outer.center();
  const double outer_r = outer.width() * 0.5;
  const double knob_r = outer_r * kKnobRadiusRatio;

  QRadialGradient base_gradient(center_pt, outer_r);
  base_gradient.setColorAt(0.0, QColor(52, 54, 58));
  base_gradient.setColorAt(0.72, QColor(36, 38, 42));
  base_gradient.setColorAt(1.0, QColor(22, 24, 28));
  painter.setPen(QPen(QColor(18, 18, 20), 2.0));
  painter.setBrush(base_gradient);
  painter.drawEllipse(outer);

  QRadialGradient rim_gradient(center_pt, outer_r);
  rim_gradient.setColorAt(0.85, QColor(0, 0, 0, 0));
  rim_gradient.setColorAt(1.0, QColor(0, 0, 0, 80));
  painter.setPen(Qt::NoPen);
  painter.setBrush(rim_gradient);
  painter.drawEllipse(outer);

  painter.setPen(QPen(QColor(70, 74, 82, 120), 1.0, Qt::DashLine));
  painter.drawLine(QPointF(center_pt.x() - outer_r * 0.72, center_pt.y()),
                   QPointF(center_pt.x() + outer_r * 0.72, center_pt.y()));
  painter.drawLine(QPointF(center_pt.x(), center_pt.y() - outer_r * 0.72),
                   QPointF(center_pt.x(), center_pt.y() + outer_r * 0.72));

  const double deadzone_r = outer_r * kDeadzoneRatio;
  painter.setPen(QPen(QColor(90, 96, 108, 90), 1.0));
  painter.setBrush(Qt::NoBrush);
  painter.drawEllipse(center_pt, deadzone_r, deadzone_r);

  const double travel = outer_r * (1.0 - kKnobRadiusRatio - 0.04);
  const QPointF knob_center(center_pt.x() + knob_.x() * travel,
                            center_pt.y() + knob_.y() * travel);

  QRadialGradient knob_gradient(knob_center - QPointF(knob_r * 0.25, knob_r * 0.25),
                                knob_r * 1.4);
  knob_gradient.setColorAt(0.0, QColor(120, 188, 255));
  knob_gradient.setColorAt(0.55, QColor(58, 132, 220));
  knob_gradient.setColorAt(1.0, QColor(28, 72, 140));
  painter.setPen(QPen(QColor(170, 210, 255, 180), 1.2));
  painter.setBrush(knob_gradient);
  painter.drawEllipse(knob_center, knob_r, knob_r);

  painter.setPen(QPen(QColor(255, 255, 255, 40), 1.0));
  painter.setBrush(Qt::NoBrush);
  painter.drawEllipse(knob_center, knob_r * 0.55, knob_r * 0.55);

  QFont label_font = font();
  label_font.setPointSize(std::max(9, label_font.pointSize()));
  label_font.setWeight(QFont::DemiBold);
  painter.setFont(label_font);
  painter.setPen(QColor(190, 194, 202));
  const QRect label_rect(0, height() - 28, width(), 24);
  painter.drawText(label_rect, Qt::AlignHCenter | Qt::AlignVCenter, label_);
}

void TeleopJoystickWidget::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  update();
}

void TeleopJoystickWidget::mousePressEvent(QMouseEvent* event) {
  if (event->button() != Qt::LeftButton) {
      return;
  }
  dragging_ = true;
  grabMouse();
  updateFromPosition(event->position());
  event->accept();
}

void TeleopJoystickWidget::mouseMoveEvent(QMouseEvent* event) {
  if (!dragging_) {
    return;
  }
  updateFromPosition(event->position());
  event->accept();
}

void TeleopJoystickWidget::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() != Qt::LeftButton || !dragging_) {
    return;
  }
  dragging_ = false;
  releaseMouse();
  knob_ = QPointF(0.0, 0.0);
  update();
  emit valueChanged(0.0, 0.0);
  emit released();
  event->accept();
}

}  // namespace teleop
}  // namespace autoviz
